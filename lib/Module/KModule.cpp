//===-- KModule.cpp -------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "KModule"

#include "Passes.h"

#include "klee/Config/Version.h"
#include "klee/Core/Interpreter.h"
#include "klee/Support/OptionCategories.h"
#include "klee/Module/Cell.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Support/Debug.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/Support/ModuleUtil.h"

#if LLVM_VERSION_CODE >= LLVM_VERSION(4, 0)
#include "llvm/Bitcode/BitcodeWriter.h"
#else
#include "llvm/Bitcode/ReaderWriter.h"
#endif
#include "llvm/IR/CallSite.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/ValueSymbolTable.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Transforms/Scalar.h"
#if LLVM_VERSION_CODE >= LLVM_VERSION(8, 0)
#include "llvm/Transforms/Scalar/Scalarizer.h"
#endif
#include "llvm/Transforms/Utils/Cloning.h"
#if LLVM_VERSION_CODE >= LLVM_VERSION(7, 0)
#include "llvm/Transforms/Utils.h"
#endif
#include "llvm/IR/Dominators.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ADT/SmallVector.h"

#include <sstream>

using namespace llvm;
using namespace klee;

namespace klee {
cl::OptionCategory
    ModuleCat("Module-related options",
              "These options affect the compile-time processing of the code.");
}

namespace {
  enum SwitchImplType {
    eSwitchTypeSimple,
    eSwitchTypeLLVM,
    eSwitchTypeInternal
  };

  enum LoopLimitType {
    LoopLimitNone,
    LoopLimitSingleExit,
    LoopLimitNoCall,
  };

  cl::opt<bool>
  OutputSource("output-source",
               cl::desc("Write the assembly for the final transformed source (default=true)"),
               cl::init(true),
	       cl::cat(ModuleCat));

  cl::opt<bool>
  OutputModule("output-module",
               cl::desc("Write the bitcode for the final transformed module"),
               cl::init(false),
	       cl::cat(ModuleCat));

  cl::opt<SwitchImplType>
  SwitchType("switch-type", cl::desc("Select the implementation of switch (default=internal)"),
             cl::values(clEnumValN(eSwitchTypeSimple, "simple", 
                                   "lower to ordered branches"),
                        clEnumValN(eSwitchTypeLLVM, "llvm", 
                                   "lower using LLVM"),
                        clEnumValN(eSwitchTypeInternal, "internal", 
                                   "execute switch internally")
                        KLEE_LLVM_CL_VAL_END),
             cl::init(eSwitchTypeInternal),
	     cl::cat(ModuleCat));
  
  cl::opt<bool>
  DebugPrintEscapingFunctions("debug-print-escaping-functions", 
                              cl::desc("Print functions whose address is taken (default=false)"),
			      cl::cat(ModuleCat));

  // Don't run VerifierPass when checking module
  cl::opt<bool>
  DontVerify("disable-verify",
             cl::desc("Do not verify the module integrity (default=false)"),
             cl::init(false), cl::cat(klee::ModuleCat));

  cl::opt<bool>
  OptimiseKLEECall("klee-call-optimisation",
                             cl::desc("Allow optimization of functions that "
                                      "contain KLEE calls (default=true)"),
                             cl::init(true), cl::cat(ModuleCat));

  cl::opt<LoopLimitType>
  LoopLimit("loop-limit",
            cl::desc(""),
            cl::values(clEnumValN(LoopLimitNone, "none", ""),
                       clEnumValN(LoopLimitSingleExit, "single-exit", ""),
                       clEnumValN(LoopLimitNoCall, "no-call", "")
                       KLEE_LLVM_CL_VAL_END),
            cl::init(LoopLimitNoCall),
            cl::cat(ModuleCat));

  cl::opt<bool>
  UseCFGPass("use-cfg-pass", cl::desc(""), cl::init(true), cl::cat(ModuleCat));

  cl::opt<std::string> AbortLocations("abort-locations",
                                      cl::desc("..."));

  cl::opt<std::string> AllowedFunctionsInLoops("allowed-functions-in-loops",
                                               cl::desc("..."));

  cl::opt<bool> ProcessSubLoops("process-sub-loops", cl::desc(""), cl::init(false), cl::cat(ModuleCat));
}

/***/

namespace llvm {
extern void Optimize(Module *, llvm::ArrayRef<const char *> preservedFunctions);
}

struct LocationOption {
  std::string filename;
  std::set<unsigned int> lines;

  LocationOption(const std::string &filename,
                 const std::set<unsigned int> &lines) :
      filename(filename), lines(lines)
  {

  }
};

static bool parseNameLineOption(std::string option,
                                std::string &filename,
                                std::set<unsigned int> &lines) {
  std::istringstream stream(option);
  std::string token;
  char *endptr;

  if (std::getline(stream, token, ':')) {
    filename = token;
    while (std::getline(stream, token, '/')) {
      /* TODO: handle errors */
      const char *s = token.c_str();
      unsigned int line = strtol(s, &endptr, 10);
      if ((errno == ERANGE) || (endptr == s) || (*endptr != '\0')) {
        return false;
      }

      lines.insert(line);
    }
  }

  return true;
}

static void parseLocationListParameter(std::string parameter,
                                       std::vector<LocationOption> &result) {
  std::istringstream stream(parameter);
  std::string token;
  std::string filename;

  while (std::getline(stream, token, ',')) {
    std::set<unsigned int> lines;
    if (!parseNameLineOption(token, filename, lines)) {
      klee_error("invalid parameter: %s", token.c_str());
    }

    result.push_back(LocationOption(filename, lines));
  }
}

static bool shouldAbortOn(KInstruction *ki,
                          const std::vector<LocationOption> &abortLocations) {
  for (const LocationOption &option : abortLocations) {
    std::string fullpath = ki->info->file;
    std::string basename = fullpath.substr(fullpath.find_last_of("/") + 1);
    if (basename == option.filename &&
        option.lines.find(ki->info->line) != option.lines.end()) {
      return true;
    }
  }
  return false;
}

static std::set<std::string> allowedFunctions;

static void parseAllowedFunctionsInLoops(std::string parameter,
                                         std::set<std::string> &result) {
  std::istringstream stream(parameter);
  std::string token;

  while (std::getline(stream, token, ',')) {
    result.insert(token);
  }
}

// what a hack
static Function *getStubFunctionForCtorList(Module *m,
                                            GlobalVariable *gv, 
                                            std::string name) {
  assert(!gv->isDeclaration() && !gv->hasInternalLinkage() &&
         "do not support old LLVM style constructor/destructor lists");

  std::vector<Type *> nullary;

  Function *fn = Function::Create(FunctionType::get(Type::getVoidTy(m->getContext()),
						    nullary, false),
				  GlobalVariable::InternalLinkage, 
				  name,
                              m);
  BasicBlock *bb = BasicBlock::Create(m->getContext(), "entry", fn);
  llvm::IRBuilder<> Builder(bb);

  // From lli:
  // Should be an array of '{ int, void ()* }' structs.  The first value is
  // the init priority, which we ignore.
  auto arr = dyn_cast<ConstantArray>(gv->getInitializer());
  if (arr) {
    for (unsigned i=0; i<arr->getNumOperands(); i++) {
      auto cs = cast<ConstantStruct>(arr->getOperand(i));
      // There is a third element in global_ctor elements (``i8 @data``).
#if LLVM_VERSION_CODE >= LLVM_VERSION(9, 0)
      assert(cs->getNumOperands() == 3 &&
             "unexpected element in ctor initializer list");
#else
      // before LLVM 9.0, the third operand was optional
      assert((cs->getNumOperands() == 2 || cs->getNumOperands() == 3) &&
             "unexpected element in ctor initializer list");
#endif
      auto fp = cs->getOperand(1);
      if (!fp->isNullValue()) {
        if (auto ce = dyn_cast<llvm::ConstantExpr>(fp))
          fp = ce->getOperand(0);

        if (auto f = dyn_cast<Function>(fp)) {
          Builder.CreateCall(f);
        } else {
          assert(0 && "unable to get function pointer from ctor initializer list");
        }
      }
    }
  }

  Builder.CreateRetVoid();

  return fn;
}

static void
injectStaticConstructorsAndDestructors(Module *m,
                                       llvm::StringRef entryFunction) {
  GlobalVariable *ctors = m->getNamedGlobal("llvm.global_ctors");
  GlobalVariable *dtors = m->getNamedGlobal("llvm.global_dtors");

  if (!ctors && !dtors)
    return;

  Function *mainFn = m->getFunction(entryFunction);
  if (!mainFn)
    klee_error("Entry function '%s' not found in module.",
               entryFunction.str().c_str());

  if (ctors) {
    llvm::IRBuilder<> Builder(&*mainFn->begin()->begin());
    Builder.CreateCall(getStubFunctionForCtorList(m, ctors, "klee.ctor_stub"));
  }

  if (dtors) {
    Function *dtorStub = getStubFunctionForCtorList(m, dtors, "klee.dtor_stub");
    for (Function::iterator it = mainFn->begin(), ie = mainFn->end(); it != ie;
         ++it) {
      if (isa<ReturnInst>(it->getTerminator())) {
        llvm::IRBuilder<> Builder(it->getTerminator());
        Builder.CreateCall(dtorStub);
      }
    }
  }
}

void KModule::addInternalFunction(const char* functionName){
  Function* internalFunction = module->getFunction(functionName);
  if (!internalFunction) {
    KLEE_DEBUG(klee_warning(
        "Failed to add internal function %s. Not found.", functionName));
    return ;
  }
  KLEE_DEBUG(klee_message("Added function %s.",functionName));
  internalFunctions.insert(internalFunction);
}

bool KModule::link(std::vector<std::unique_ptr<llvm::Module>> &modules,
                   const std::string &entryPoint) {
  auto numRemainingModules = modules.size();
  // Add the currently active module to the list of linkables
  modules.push_back(std::move(module));
  std::string error;
  module = std::unique_ptr<llvm::Module>(
      klee::linkModules(modules, entryPoint, error));
  if (!module)
    klee_error("Could not link KLEE files %s", error.c_str());

  targetData = std::unique_ptr<llvm::DataLayout>(new DataLayout(module.get()));

  // Check if we linked anything
  return modules.size() != numRemainingModules;
}

void KModule::instrument(const Interpreter::ModuleOptions &opts) {
  // Inject checks prior to optimization... we also perform the
  // invariant transformations that we will end up doing later so that
  // optimize is seeing what is as close as possible to the final
  // module.
  legacy::PassManager pm;
  pm.add(new RaiseAsmPass());

  // This pass will scalarize as much code as possible so that the Executor
  // does not need to handle operands of vector type for most instructions
  // other than InsertElementInst and ExtractElementInst.
  //
  // NOTE: Must come before division/overshift checks because those passes
  // don't know how to handle vector instructions.
  pm.add(createScalarizerPass());

  // This pass will replace atomic instructions with non-atomic operations
  pm.add(createLowerAtomicPass());
  if (opts.CheckDivZero) pm.add(new DivCheckPass());
  if (opts.CheckOvershift) pm.add(new OvershiftCheckPass());

  pm.add(new IntrinsicCleanerPass(*targetData));
  pm.run(*module);
}

void KModule::optimiseAndPrepare(
    const Interpreter::ModuleOptions &opts,
    llvm::ArrayRef<const char *> preservedFunctions) {
  // Preserve all functions containing klee-related function calls from being
  // optimised around
  if (!OptimiseKLEECall) {
    legacy::PassManager pm;
    pm.add(new OptNonePass());
    pm.run(*module);
  }

  if (opts.Optimize)
    Optimize(module.get(), preservedFunctions);

  // Add internal functions which are not used to check if instructions
  // have been already visited
  if (opts.CheckDivZero)
    addInternalFunction("klee_div_zero_check");
  if (opts.CheckOvershift)
    addInternalFunction("klee_overshift_check");

  // Needs to happen after linking (since ctors/dtors can be modified)
  // and optimization (since global optimization can rewrite lists).
  injectStaticConstructorsAndDestructors(module.get(), opts.EntryPoint);

  // Finally, run the passes that maintain invariants we expect during
  // interpretation. We run the intrinsic cleaner just in case we
  // linked in something with intrinsics but any external calls are
  // going to be unresolved. We really need to handle the intrinsics
  // directly I think?
  legacy::PassManager pm3;
  if (UseCFGPass) {
    /* TODO: this might make incremental merging hard, check... */
    pm3.add(createCFGSimplificationPass());
  }
  switch(SwitchType) {
  case eSwitchTypeInternal: break;
  case eSwitchTypeSimple: pm3.add(new LowerSwitchPass()); break;
  case eSwitchTypeLLVM:  pm3.add(createLowerSwitchPass()); break;
  default: klee_error("invalid --switch-type");
  }
  pm3.add(new IntrinsicCleanerPass(*targetData));
  pm3.add(createScalarizerPass());
  pm3.add(new PhiCleanerPass());
  pm3.add(new FunctionAliasPass());
  pm3.run(*module);
}

void KModule::manifest(InterpreterHandler *ih, bool forceSourceOutput) {
  if (OutputSource || forceSourceOutput) {
    std::unique_ptr<llvm::raw_fd_ostream> os(ih->openOutputFile("assembly.ll"));
    assert(os && !os->has_error() && "unable to open source output");
    *os << *module;
  }

  if (OutputModule) {
    std::unique_ptr<llvm::raw_fd_ostream> f(ih->openOutputFile("final.bc"));
#if LLVM_VERSION_CODE >= LLVM_VERSION(7, 0)
    WriteBitcodeToFile(*module, *f);
#else
    WriteBitcodeToFile(module.get(), *f);
#endif
  }

  /* Build shadow structures */

  infos = std::unique_ptr<InstructionInfoTable>(
      new InstructionInfoTable(*module.get()));

  std::vector<Function *> declarations;

  std::vector<LocationOption> abortLocations;
  parseLocationListParameter(AbortLocations, abortLocations);

  parseAllowedFunctionsInLoops(AllowedFunctionsInLoops, allowedFunctions);

  for (auto &Function : *module) {
    if (Function.isDeclaration()) {
      declarations.push_back(&Function);
      continue;
    }

    auto kf = std::unique_ptr<KFunction>(new KFunction(&Function, this));

    for (unsigned i=0; i<kf->numInstructions; ++i) {
      KInstruction *ki = kf->instructions[i];
      ki->info = &infos->getInfo(*ki->inst);
      ki->shouldAbort = shouldAbortOn(ki, abortLocations);
    }

    functionMap.insert(std::make_pair(&Function, kf.get()));
    functions.push_back(std::move(kf));
  }

  /* Compute various interesting properties */

  for (auto &kf : functions) {
    if (functionEscapes(kf->function))
      escapingFunctions.insert(kf->function);
  }

  for (auto &declaration : declarations) {
    if (functionEscapes(declaration))
      escapingFunctions.insert(declaration);
  }

  if (DebugPrintEscapingFunctions && !escapingFunctions.empty()) {
    llvm::errs() << "KLEE: escaping functions: [";
    std::string delimiter = "";
    for (auto &Function : escapingFunctions) {
      llvm::errs() << delimiter << Function->getName();
      delimiter = ", ";
    }
    llvm::errs() << "]\n";
  }

  collectLoopInfo();
}

void KModule::collectLoopInfo() {
  for (Function &f : *module) {
    if (f.isDeclaration()) {
      continue;
    }

    llvm::DominatorTree dom;
    dom.recalculate(f);
    llvm::LoopInfo* li = new llvm::LoopInfo(dom);
    loopInfos.push_back(li);
    for (Loop *loop : *li) {
      visitLoop(f, loop);
      if (ProcessSubLoops) {
        for (Loop *subLoop : loop->getSubLoops()) {
          visitLoop(f, subLoop);
        }
      }
    }
  }
}

/* TODO: is it the correct solution? */
/* TODO: add 'isSupported' attribute? */
void KModule::visitLoop(Function &f, Loop *loop) {
  bool isSupported = isSupportedLoop(loop);
  KLoop kloop(loop, isSupported);

  /* TODO: mark all basic block? */
  Instruction *term = loop->getHeader()->getTerminator();
  KInstruction *ki = instructionMap[term];
  ki->entryLoop = kloop;
  ki->isLoopEntry = true;

  SmallVector<BasicBlock *, 10> blocks;
  loop->getExitBlocks(blocks);
  for (BasicBlock *bb : blocks) {
    /* TODO: mark all basic block? */
    Instruction &first = bb->front();
    KInstruction *kfirst = instructionMap[&first];
    kfirst->exitLoops.push_back(kloop);
    kfirst->isLoopExit = true;
  }
}

bool KModule::isSupportedLoop(Loop *loop) {
  std::set<std::string> wl;
  wl.insert("strncasecmp_l");
  wl.insert("strncasecmp");
  if (wl.find(loop->getHeader()->getParent()->getName()) != wl.end()) {
    return true;
  }

  std::set<std::string> bl;
  if (bl.find(loop->getHeader()->getParent()->getName()) != bl.end()) {
    return false;
  }

  SmallVector<BasicBlock *, 10> blocks;
  switch (LoopLimit) {
    case LoopLimitNone:
      return true;

    case LoopLimitSingleExit:
      loop->getExitBlocks(blocks);
      return blocks.size() == 1;

    case LoopLimitNoCall:
      return !hasFunctionCalls(loop);

    default:
      break;
  }

  assert(0);
}

static std::string getOriginalFunctionName(Function *f) {
  DISubprogram *di = f->getSubprogram();
  if (di) {
    return di->getName();
  } else {
    return f->getName();
  }
}

bool KModule::hasFunctionCalls(Loop *loop) {
  for (BasicBlock *bb : loop->getBlocks()) {
    for (Instruction &inst : *bb) {
      CallInst *call = dyn_cast<CallInst>(&inst);
      if (call) {
        Function *f = call->getCalledFunction();
        if (f) {
          if (!f->isDeclaration() || !f->isIntrinsic()) {
            std::string name = getOriginalFunctionName(f);
            if (allowedFunctions.find(name) == allowedFunctions.end()) {
              return true;
            }
          }
        } else {
          /* virtual call */
          return true;
        }
      }
    }
  }
  return false;
}

void KModule::checkModule() {
  InstructionOperandTypeCheckPass *operandTypeCheckPass =
      new InstructionOperandTypeCheckPass();

  legacy::PassManager pm;
  if (!DontVerify)
    pm.add(createVerifierPass());
  pm.add(operandTypeCheckPass);
  pm.run(*module);

  // Enforce the operand type invariants that the Executor expects.  This
  // implicitly depends on the "Scalarizer" pass to be run in order to succeed
  // in the presence of vector instructions.
  if (!operandTypeCheckPass->checkPassed()) {
    klee_error("Unexpected instruction operand types detected");
  }
}

KConstant* KModule::getKConstant(const Constant *c) {
  auto it = constantMap.find(c);
  if (it != constantMap.end())
    return it->second.get();
  return NULL;
}

unsigned KModule::getConstantID(Constant *c, KInstruction* ki) {
  if (KConstant *kc = getKConstant(c))
    return kc->id;  

  unsigned id = constants.size();
  auto kc = std::unique_ptr<KConstant>(new KConstant(c, id, ki));
  constantMap.insert(std::make_pair(c, std::move(kc)));
  constants.push_back(c);
  return id;
}

KModule::~KModule() {
  for (llvm::LoopInfo *li : loopInfos) {
    delete li;
  }
}

/***/

KConstant::KConstant(llvm::Constant* _ct, unsigned _id, KInstruction* _ki) {
  ct = _ct;
  id = _id;
  ki = _ki;
}

/***/

static int getOperandNum(Value *v,
                         std::map<Instruction*, unsigned> &registerMap,
                         KModule *km,
                         KInstruction *ki) {
  if (Instruction *inst = dyn_cast<Instruction>(v)) {
    return registerMap[inst];
  } else if (Argument *a = dyn_cast<Argument>(v)) {
    return a->getArgNo();
  } else if (isa<BasicBlock>(v) || isa<InlineAsm>(v) ||
             isa<MetadataAsValue>(v)) {
    return -1;
  } else {
    assert(isa<Constant>(v));
    Constant *c = cast<Constant>(v);
    return -(km->getConstantID(c, ki) + 2);
  }
}

KFunction::KFunction(llvm::Function *_function,
                     KModule *km) 
  : function(_function),
    numArgs(function->arg_size()),
    numInstructions(0),
    trackCoverage(true) {
  // Assign unique instruction IDs to each basic block
  for (auto &BasicBlock : *function) {
    basicBlockEntry[&BasicBlock] = numInstructions;
    numInstructions += BasicBlock.size();
  }

  instructions = new KInstruction*[numInstructions];

  std::map<Instruction*, unsigned> registerMap;

  // The first arg_size() registers are reserved for formals.
  unsigned rnum = numArgs;
  for (llvm::Function::iterator bbit = function->begin(), 
         bbie = function->end(); bbit != bbie; ++bbit) {
    for (llvm::BasicBlock::iterator it = bbit->begin(), ie = bbit->end();
         it != ie; ++it) {
      Instruction *inst = &*it;
      registerMap[inst] = rnum;
      inverseRegisterMap[rnum] = inst;
      rnum++;
    }
  }
  numRegisters = rnum;
  
  unsigned i = 0;
  for (llvm::Function::iterator bbit = function->begin(), 
         bbie = function->end(); bbit != bbie; ++bbit) {
    for (llvm::BasicBlock::iterator it = bbit->begin(), ie = bbit->end();
         it != ie; ++it) {
      KInstruction *ki;

      switch(it->getOpcode()) {
      case Instruction::GetElementPtr:
      case Instruction::InsertValue:
      case Instruction::ExtractValue:
        ki = new KGEPInstruction(); break;
      default:
        ki = new KInstruction(); break;
      }

      Instruction *inst = &*it;
      ki->inst = inst;
      ki->dest = registerMap[inst];

      if (isa<CallInst>(it) || isa<InvokeInst>(it)) {
        CallSite cs(inst);
        unsigned numArgs = cs.arg_size();
        ki->operands = new int[numArgs+1];
        ki->operands[0] = getOperandNum(cs.getCalledValue(), registerMap, km,
                                        ki);
        for (unsigned j=0; j<numArgs; j++) {
          Value *v = cs.getArgument(j);
          ki->operands[j+1] = getOperandNum(v, registerMap, km, ki);
        }
      } else {
        unsigned numOperands = it->getNumOperands();
        ki->operands = new int[numOperands];
        for (unsigned j=0; j<numOperands; j++) {
          Value *v = it->getOperand(j);
          ki->operands[j] = getOperandNum(v, registerMap, km, ki);
        }
      }

      instructions[i++] = ki;
      km->instructionMap[inst] = ki;
    }
  }
}

KFunction::~KFunction() {
  for (unsigned i=0; i<numInstructions; ++i)
    delete instructions[i];
  delete[] instructions;
}
