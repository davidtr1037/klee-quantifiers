//===-- ExecutionState.cpp ------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "ExecutionState.h"
#include "Memory.h"
#include "MergeUtils.h"
#include "CoreStats.h"
#include "PatternExtraction.h"
#include "Quantification.h"
#include "LivenessAnalysis.h"

#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/ExprRename.h"
#include "klee/Expr/EMatching.h"
#include "klee/Module/Cell.h"
#include "klee/Module/InstructionInfoTable.h"
#include "klee/Module/KInstruction.h"
#include "klee/Module/KModule.h"
#include "klee/Support/OptionCategories.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/Solver/SolverStats.h"
#include "klee/Statistics/TimerStatIncrementer.h"

#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/raw_ostream.h"

#include <cassert>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdarg.h>

using namespace llvm;
using namespace klee;

namespace {
cl::opt<bool> DebugLogStateMerge(
    "debug-log-state-merge", cl::init(false),
    cl::desc("Debug information for underlying state merging (default=false)"),
    cl::cat(MergeCat));

/* TODO: can't be used with -validate-merge */
cl::opt<bool> OptimizeArrayValuesUsingITERewrite(
    "optimize-array-values-using-ite-rewrite", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

/* TODO: can't be used with -validate-merge */
cl::opt<bool> OptimizeArrayValuesUsingSolver(
    "optimize-array-values-using-solver", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));
}

cl::opt<bool> klee::OptimizeITEUsingExecTree(
    "optimize-ite-using-exec-tree", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> klee::OptimizeArrayITEUsingExecTree(
    "optimize-array-ite-using-exec-tree", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

/* TODO: can't be used with -validate-merge */
cl::opt<bool> klee::OptimizeArrayValuesByTracking(
    "optimize-array-values-by-tracking", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> DumpExecutionTree(
    "dump-exec-tree", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

/* TODO: rename to UseQuantifiers? */
cl::opt<bool> klee::OptimizeUsingQuantifiers(
    "optimize-using-quantifiers", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> klee::CreateSnapshots(
    "create-snapshots", cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> UseLocalMergeID(
    "use-local-merge-id",
    cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> klee::RewriteExpr(
    "rewrite-expr",
    cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> SimplifyITE(
    "simplify-ite",
    cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> GenerateLemmasOnFork(
    "generate-lemmas-on-fork",
    cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

cl::opt<bool> GenerateLemmasOnMerge(
    "generate-lemmas-on-merge",
    cl::init(false),
    cl::desc(""),
    cl::cat(MergeCat));

/***/

std::uint32_t ExecutionState::nextID = 1;
std::uint32_t ExecutionState::mergeID = 1;

/***/

StackFrame::StackFrame(KInstIterator _caller, KFunction *_kf)
  : caller(_caller), kf(_kf), callPathNode(0), 
    minDistToUncoveredOnReturn(0), varargs(0), isExecutingLoop(false) {
  locals = new Cell[kf->numRegisters];
}

StackFrame::StackFrame(KInstIterator _caller, KFunction *_kf, bool isExecutingLoop, KLoop loop)
  : caller(_caller), kf(_kf), callPathNode(0),
    minDistToUncoveredOnReturn(0), varargs(0), isExecutingLoop(isExecutingLoop), loop(loop) {
  locals = new Cell[kf->numRegisters];
}

StackFrame::StackFrame(const StackFrame &s) 
  : caller(s.caller),
    kf(s.kf),
    callPathNode(s.callPathNode),
    allocas(s.allocas),
    minDistToUncoveredOnReturn(s.minDistToUncoveredOnReturn),
    varargs(s.varargs),
    isExecutingLoop(s.isExecutingLoop),
    loop(s.loop) {
  locals = new Cell[s.kf->numRegisters];
  for (unsigned i=0; i<s.kf->numRegisters; i++)
    locals[i] = s.locals[i];
}

StackFrame::~StackFrame() { 
  delete[] locals; 
}

/***/

ExecutionState::ExecutionState(KFunction *kf) :
    pc(kf->instructions),
    prevPC(pc),
    depth(0),
    ptreeNode(nullptr),
    steppedInstructions(0),
    instsSinceCovNew(0),
    coveredNew(false),
    forkDisabled(false),
    loopHandler(nullptr),
    isSnapshot(false),
    hasPendingSnapshot(false),
    localMergeID(0) {
  pushFrame(nullptr, kf);
  setID();
}

ExecutionState::~ExecutionState() {
  for (const auto &cur_mergehandler: openMergeStack){
    cur_mergehandler->removeOpenState(this);
  }
  if (!loopHandler.isNull() && !isSnapshot) {
    /* TODO: mark here as incomplete execution? */
    loopHandler->removeOpenState(this);
  }

  while (!stack.empty()) popFrame();
}

/* TODO: try to avoid adding patameters */
ExecutionState::ExecutionState(const ExecutionState& state, bool isSnapshot):
    pc(state.pc),
    prevPC(state.prevPC),
    stack(state.stack),
    incomingBBIndex(state.incomingBBIndex),
    depth(state.depth),
    addressSpace(state.addressSpace),
    constraints(state.constraints),
    rewrittenConstraints(state.rewrittenConstraints),
    pathOS(state.pathOS),
    symPathOS(state.symPathOS),
    coveredLines(state.coveredLines),
    symbolics(state.symbolics),
    arrayNames(state.arrayNames),
    openMergeStack(state.openMergeStack),
    steppedInstructions(state.steppedInstructions),
    instsSinceCovNew(state.instsSinceCovNew),
    coveredNew(state.coveredNew),
    forkDisabled(state.forkDisabled),
    /* TODO: copy-on-write? */
    taintedExprs(state.taintedExprs),
    loopHandler(state.loopHandler),
    suffixConstraints(state.suffixConstraints),
    isSnapshot(isSnapshot),
    hasPendingSnapshot(false),
    localMergeID(state.localMergeID),
    renamingMap(state.renamingMap) {
  for (const auto &cur_mergehandler: openMergeStack) {
    cur_mergehandler->addOpenState(this);
  }
  if (!loopHandler.isNull() && !isSnapshot) {
    loopHandler->addOpenState(this);
  }
  if (!isSnapshot) {
    /* make sure we don't have unexpected forks */
    if (state.hasPendingSnapshot) {
      assert(!loopHandler.isNull() && !loopHandler->canUseExecTree);
    }
  }
}

ExecutionState *ExecutionState::branch(bool isSnapshot) {
  depth++;

  auto *falseState = new ExecutionState(*this, isSnapshot);
  falseState->setID();
  falseState->coveredNew = false;
  falseState->coveredLines.clear();

  return falseState;
}

void ExecutionState::pushFrame(KInstIterator caller, KFunction *kf) {
  if (stack.empty()) {
    stack.emplace_back(StackFrame(caller, kf));
  } else {
    stack.emplace_back(StackFrame(caller, kf, stack.back().isExecutingLoop, stack.back().loop));
  }
}

void ExecutionState::popFrame() {
  const StackFrame &sf = stack.back();
  for (const auto * memoryObject : sf.allocas)
    addressSpace.unbindObject(memoryObject);
  stack.pop_back();
}

void ExecutionState::addSymbolic(const MemoryObject *mo, const Array *array) {
  symbolics.emplace_back(ref<const MemoryObject>(mo), array);
}

/**/

llvm::raw_ostream &klee::operator<<(llvm::raw_ostream &os, const MemoryMap &mm) {
  os << "{";
  MemoryMap::iterator it = mm.begin();
  MemoryMap::iterator ie = mm.end();
  if (it!=ie) {
    os << "MO" << it->first->id << ":" << it->second.get();
    for (++it; it!=ie; ++it)
      os << ", MO" << it->first->id << ":" << it->second.get();
  }
  os << "}";
  return os;
}

bool ExecutionState::merge(const ExecutionState &b) {
  if (DebugLogStateMerge)
    llvm::errs() << "-- attempting merge of A:" << this << " with B:" << &b
                 << "--\n";
  if (pc != b.pc)
    return false;

  // XXX is it even possible for these to differ? does it matter? probably
  // implies difference in object states?

  if (symbolics != b.symbolics)
    return false;

  {
    std::vector<StackFrame>::const_iterator itA = stack.begin();
    std::vector<StackFrame>::const_iterator itB = b.stack.begin();
    while (itA!=stack.end() && itB!=b.stack.end()) {
      // XXX vaargs?
      if (itA->caller!=itB->caller || itA->kf!=itB->kf)
        return false;
      ++itA;
      ++itB;
    }
    if (itA!=stack.end() || itB!=b.stack.end())
      return false;
  }

  std::set< ref<Expr> > aConstraints(constraints.begin(), constraints.end());
  std::set< ref<Expr> > bConstraints(b.constraints.begin(), 
                                     b.constraints.end());
  std::set< ref<Expr> > commonConstraints, aSuffix, bSuffix;
  std::set_intersection(aConstraints.begin(), aConstraints.end(),
                        bConstraints.begin(), bConstraints.end(),
                        std::inserter(commonConstraints, commonConstraints.begin()));
  std::set_difference(aConstraints.begin(), aConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(aSuffix, aSuffix.end()));
  std::set_difference(bConstraints.begin(), bConstraints.end(),
                      commonConstraints.begin(), commonConstraints.end(),
                      std::inserter(bSuffix, bSuffix.end()));
  if (DebugLogStateMerge) {
    llvm::errs() << "\tconstraint prefix: [";
    for (std::set<ref<Expr> >::iterator it = commonConstraints.begin(),
                                        ie = commonConstraints.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tA suffix: [";
    for (std::set<ref<Expr> >::iterator it = aSuffix.begin(),
                                        ie = aSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
    llvm::errs() << "\tB suffix: [";
    for (std::set<ref<Expr> >::iterator it = bSuffix.begin(),
                                        ie = bSuffix.end();
         it != ie; ++it)
      llvm::errs() << *it << ", ";
    llvm::errs() << "]\n";
  }

  // We cannot merge if addresses would resolve differently in the
  // states. This means:
  // 
  // 1. Any objects created since the branch in either object must
  // have been free'd.
  //
  // 2. We cannot have free'd any pre-existing object in one state
  // and not the other

  if (DebugLogStateMerge) {
    llvm::errs() << "\tchecking object states\n";
    llvm::errs() << "A: " << addressSpace.objects << "\n";
    llvm::errs() << "B: " << b.addressSpace.objects << "\n";
  }
    
  std::set<const MemoryObject*> mutated;
  MemoryMap::iterator ai = addressSpace.objects.begin();
  MemoryMap::iterator bi = b.addressSpace.objects.begin();
  MemoryMap::iterator ae = addressSpace.objects.end();
  MemoryMap::iterator be = b.addressSpace.objects.end();
  for (; ai!=ae && bi!=be; ++ai, ++bi) {
    if (ai->first != bi->first) {
      if (DebugLogStateMerge) {
        if (ai->first < bi->first) {
          llvm::errs() << "\t\tB misses binding for: " << ai->first->id << "\n";
        } else {
          llvm::errs() << "\t\tA misses binding for: " << bi->first->id << "\n";
        }
      }
      return false;
    }
    if (ai->second.get() != bi->second.get()) {
      if (DebugLogStateMerge)
        llvm::errs() << "\t\tmutated: " << ai->first->id << "\n";
      mutated.insert(ai->first);
    }
  }
  if (ai!=ae || bi!=be) {
    if (DebugLogStateMerge)
      llvm::errs() << "\t\tmappings differ\n";
    return false;
  }

  if (RewriteExpr) {
    /* handle auxiliary variables */
    for (auto i : b.renamingMap) {
      const Array *array = i.first;
      renamingMap[array] = nullptr;
    }
    mapAuxArrays();
  }
  
  // merge stack

  ref<Expr> inA = ConstantExpr::alloc(1, Expr::Bool);
  ref<Expr> inB = ConstantExpr::alloc(1, Expr::Bool);
  for (ref<Expr> e : constraints) {
    if (aSuffix.find(e) != aSuffix.end()) {
      inA = AndExpr::create(inA, e);
    }
  }
  for (ref<Expr> e : b.constraints) {
    if (bSuffix.find(e) != bSuffix.end()) {
      inB = AndExpr::create(inB, e);
    }
  }

  // XXX should we have a preference as to which predicate to use?
  // it seems like it can make a difference, even though logically
  // they must contradict each other and so inA => !inB

  std::vector<StackFrame>::iterator itA = stack.begin();
  std::vector<StackFrame>::const_iterator itB = b.stack.begin();
  for (; itA!=stack.end(); ++itA, ++itB) {
    StackFrame &af = *itA;
    const StackFrame &bf = *itB;
    for (unsigned i=0; i<af.kf->numRegisters; i++) {
      ref<Expr> &av = af.locals[i].value;
      const ref<Expr> &bv = bf.locals[i].value;
      if (av.isNull() || bv.isNull()) {
        // if one is null then by implication (we are at same pc)
        // we cannot reuse this local, so just ignore
        /* TODO: for consistency */
        av = nullptr;
      } else {
        av = SelectExpr::create(inA, av, bv);
      }
    }
  }

  for (std::set<const MemoryObject*>::iterator it = mutated.begin(), 
         ie = mutated.end(); it != ie; ++it) {
    const MemoryObject *mo = *it;
    const ObjectState *os = addressSpace.findObject(mo);
    const ObjectState *otherOS = b.addressSpace.findObject(mo);
    assert(os && !os->readOnly && 
           "objects mutated but not writable in merging state");
    assert(otherOS);
    assert(os->getObject()->capacity == otherOS->getObject()->capacity);

    ObjectState *wos = addressSpace.getWriteable(mo, os);
    for (unsigned i = 0; i < mo->capacity; i++) {
      ref<Expr> av = wos->read8(i);
      ref<Expr> bv = otherOS->read8(i);
      ref<Expr> cv = SelectExpr::create(inA, av, bv);
      wos->write(i, cv);
    }
  }

  constraints = ConstraintSet();
  rewrittenConstraints = ConstraintSet();
  for (ref<Expr> e : b.constraints) {
    if (commonConstraints.find(e) != commonConstraints.end()) {
      addConstraint(e, true);
    }
  }
  addConstraint(OrExpr::create(inA, inB), true);

  if (DebugLogStateMerge) {
    llvm::errs() << "merge succeeded\n";
  }
  return true;
}

void ExecutionState::dumpStack(llvm::raw_ostream &out) const {
  unsigned idx = 0;
  const KInstruction *target = prevPC;
  for (ExecutionState::stack_ty::const_reverse_iterator
         it = stack.rbegin(), ie = stack.rend();
       it != ie; ++it) {
    const StackFrame &sf = *it;
    Function *f = sf.kf->function;
    const InstructionInfo &ii = *target->info;
    out << "\t#" << idx++;
    std::stringstream AssStream;
    AssStream << std::setw(8) << std::setfill('0') << ii.assemblyLine;
    out << AssStream.str();
    out << " in " << f->getName().str() << " (";
    // Yawn, we could go up and print varargs if we wanted to.
    unsigned index = 0;
    for (Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
         ai != ae; ++ai) {
      if (ai!=f->arg_begin()) out << ", ";

      out << ai->getName().str();
      // XXX should go through function
      ref<Expr> value = sf.locals[sf.kf->getArgRegister(index++)].value;
      if (value.get() && isa<ConstantExpr>(value))
        out << "=" << value;
    }
    out << ")";
    if (ii.file != "")
      out << " at " << ii.file << ":" << ii.line;
    out << "\n";
    target = sf.caller;
  }
}

void ExecutionState::addConstraint(ref<Expr> e, bool atMerge) {
  ConstraintManager c(constraints);
  bool changed = c.addConstraint(e);

  std::vector<ref<Expr>> lemmas;
  if (GenerateLemmasOnFork && !atMerge && !changed) {
    for (ref<Expr> constraint : constraints) {
      if (isa<ForallExpr>(constraint)) {
        ConstraintSet tmp;
        tmp.push_back(constraints.last());
        ref<ForallExpr> f = dyn_cast<ForallExpr>(constraint);
        assert(!f.isNull());
        generateLemmaFromForall(f, tmp, false, true, lemmas);
      }
    }
  }
  for (ref<Expr> lemma : lemmas) {
    klee_message("adding lemma (addConstraint)");
    c.addConstraint(lemma);
    stats::mergedConstraintsSize += lemma->size;
  }

  if (RewriteExpr) {
    if (changed) {
      rewriteConstraints();
    } else {
      /* TODO: add the tail using a loop? */
      if (rewrittenConstraints.size() != constraints.size()) {
        ref<Expr> renamed = rename(constraints.last(), renamingMap);
        rewrittenConstraints.push_back(renamed);
      }
    }
    assert(rewrittenConstraints.size() == constraints.size());
  }

  if (!atMerge) {
    if (!loopHandler.isNull()) {
      /* we don't expect forks after the state is paused */
      assert(stack.back().isExecutingLoop);
      addSuffixConstraint(e);
    }
  }
}

void ExecutionState::addSuffixConstraint(ref<Expr> e) {
  ConstraintManager m(suffixConstraints);
  m.addConstraint(e);
}

void ExecutionState::clearSuffixConstraints() {
  suffixConstraints.clear();
}

void ExecutionState::addTaintedExpr(std::string name, ref<Expr> offset) {
  ExprSet &exprs = taintedExprs[name];
  for (ref<Expr> e : exprs) {
    if (*e == *offset) {
      return;
    }
  }

  exprs.push_back(offset);
}

/* TODO: handle symbolic offsets... */
bool ExecutionState::hasTaintedExpr(std::string name, ref<Expr> offset) {
  auto i = taintedExprs.find(name);
  if (i != taintedExprs.end()) {
    for (ref<Expr> e : i->second) {
      /* TODO: this is a conservative check, use the solver? */
      if (!isa<ConstantExpr>(offset) || !isa<ConstantExpr>(e) || *e == *offset) {
        return true;
      }
    }
  }

  return false;
}

bool ExecutionState::isTaintedExpr(ref<Expr> e) {
  TaintVisitor visitor(*this);
  visitor.visit(e);
  return visitor.isTainted;
}

ExprVisitor::Action TaintVisitor::visitRead(const ReadExpr &e) {
  if (e.hasAuxVariable) {
    isTainted = true;
    return Action::skipChildren();
  }

  for (const UpdateNode *un = e.updates.head.get(); un; un = un->next.get()) {
    /* TODO: break if found? */
    visit(un->index);
    visit(un->value);
  }

  const Array *array = e.updates.root;
  const std::string &name = array->getName();
  if (state.hasTaintedExpr(name, e.index)) {
    isTainted = true;
    return Action::skipChildren();
  } else {
    return Action::doChildren();
  }
}

ExecutionContext::ExecutionContext(ExecutionState &state, bool usePrevPC) {
  const KInstruction *kinst = usePrevPC ? state.prevPC : state.pc;
  for (auto i = state.stack.rbegin(); i != state.stack.rend(); i++) {
    StackFrame &sf = *i;
    Function *f = sf.kf->function;
    if (f->getName() == "__uClibc_main") {
      /* not interesting from that point on... */
      break;
    }

    CodeLocation location(kinst ? kinst->info->file : "unknown",
                          kinst ? kinst->info->line : 0,
                          f->getName());
    trace.push_back(location);
    kinst = sf.caller;
  }
}

void ExecutionContext::dump() const {
  for (const CodeLocation &location : trace) {
    location.dump();
  }
}

ExecutionState *ExecutionState::mergeStates(std::vector<ExecutionState *> &states) {
  assert(!states.empty());
  ExecutionState *merged = states[0];

  for (unsigned i = 1; i < states.size(); i++) {
    ExecutionState *es = states[i];
    if (!merged->merge(*es)) {
      return nullptr;
    }
  }

  return merged;
}

/* TODO: pass a single PatternMatch instead of a list? */
/* TODO: pass the merge identifier? */
ExecutionState *ExecutionState::mergeStatesOptimized(std::vector<ExecutionState *> &states,
                                                     bool isComplete,
                                                     bool usePattern,
                                                     std::vector<PatternMatch> &matches,
                                                     LoopHandler *loopHandler) {
  if (usePattern) {
    assert(matches.size() == 1);
  }

  std::set<const MemoryObject*> mutated;
  if (!canMerge(states, mutated)) {
    return nullptr;
  }

  if (DumpExecutionTree) {
    std::set<uint32_t> ids;
    for (ExecutionState *es : states) {
      ids.insert(es->getID());
    }
    std::string fname = loopHandler->loop->getHeader()->getParent()->getName();
    loopHandler->tree.dumpGMLToFile(ids, fname);
  }

  /* compute suffix for each state */
  std::vector<ref<Expr>> suffixes;
  for (unsigned i = 0; i < states.size(); i++) {
    ExecutionState *es = states[i];
    ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
    for (ref<Expr> e : es->suffixConstraints) {
      all = AndExpr::create(all, e);
    }
    suffixes.push_back(all);
  }

  ExecutionState *merged = states[0];

  if (RewriteExpr) {
    /* handle auxiliary variables */
    for (unsigned i = 0; i < states.size(); i++) {
      ExecutionState *es = states[i];
      for (auto j : es->renamingMap) {
        const Array *array = j.first;
        merged->renamingMap[array] = nullptr;
      }
    }
    merged->mapAuxArrays();
  }

  /* path constraints */
  merged->constraints = ConstraintSet();
  merged->rewrittenConstraints = ConstraintSet();
  for (ref<Expr> e : loopHandler->initialConstraints) {
    merged->addConstraint(e, true);
  }

  /* TODO: rename to usingABV */
  bool isEncodedWithABV = false;
  if (!isComplete) {
    /* TODO: rename to mergedConstraint? */
    ref<Expr> orExpr = nullptr;
    if (usePattern && matches.size() == 1) {
      orExpr = generateQuantifiedConstraint(matches[0],
                                            loopHandler->tree,
                                            merged->getMergeID(),
                                            *loopHandler->solver);
      if (orExpr.isNull()) {
        klee_message("failed to generate the merged constraint (ABV)");
      } else {
        isEncodedWithABV = true;
      }
    }
    if (orExpr.isNull()) {
      if (OptimizeITEUsingExecTree && loopHandler->canUseExecTree) {
        orExpr = mergeConstraintsWithExecTree(loopHandler, states);
      } else {
        orExpr = mergeConstraints(states);
      }
    }

    if (isEncodedWithABV) {
      /* TODO: add parameter */
      if (RewriteExpr) {
        merged->addAuxArray();
      }
    }

    if ((GenerateLemmasOnFork || GenerateLemmasOnMerge) && isEncodedWithABV) {
      ref<ForallExpr> f = findForallExpr(orExpr);
      assert(!f.isNull());

      std::vector<ref<Expr>> lemmas;
      generateLemmaFromForall(f,
                              merged->constraints,
                              true,
                              GenerateLemmasOnMerge,
                              lemmas);
      for (ref<Expr> lemma : lemmas) {
        klee_message("adding lemma (merge)");
        merged->addConstraint(lemma, true);
        stats::mergedConstraintsSize += lemma->size;
      }
    }

    merged->addConstraint(orExpr, true);
    stats::mergedConstraintsSize += orExpr->size;
    klee_message("partial merge constraint size %lu", orExpr->size);
  }

  bool usedAuxVariables;

  /* local vars */
  mergeStack(merged,
             states,
             suffixes,
             loopHandler,
             isEncodedWithABV,
             matches[0],
             usedAuxVariables);

  /* heap */
  mergeHeap(merged,
            states,
            suffixes,
            mutated,
            loopHandler,
            isEncodedWithABV,
            matches[0],
            usedAuxVariables);

  if (OptimizeArrayValuesUsingITERewrite) {
    merged->optimizeArrayValues(mutated, loopHandler->solver);
  }

  return merged;
}

bool ExecutionState::canMerge(std::vector<ExecutionState *> &states,
                              std::set<const MemoryObject*> &mutated) {
  assert(!states.empty());
  ExecutionState *merged = states[0];

  /* program counter */
  for (ExecutionState *es : states) {
    if (es->pc != merged->pc) {
      return false;
    }
  }

  /* symbolics */
  for (ExecutionState *es : states) {
    if (es->symbolics != merged->symbolics) {
      return false;
    }
  }

  /* stack */
  for (ExecutionState *es : states) {
    auto i = merged->stack.begin();
    auto j = es->stack.begin();
    while (i != merged->stack.end() && j != es->stack.end()) {
      if (i->caller != j->caller || i->kf != j->kf) {
        return false;
      }
      ++i;
      ++j;
    }
    if (i != merged->stack.end() || j != es->stack.end()) {
      return false;
    }
  }

  /* address space */
  for (ExecutionState *es : states) {
    auto ai = merged->addressSpace.objects.begin();
    auto bi = es->addressSpace.objects.begin();
    auto ae = merged->addressSpace.objects.end();
    auto be = es->addressSpace.objects.end();
    for (; ai != ae && bi != be; ++ai, ++bi) {
      if (ai->first != bi->first) {
        return false;
      }
      if (ai->second.get() != bi->second.get()) {
        mutated.insert(ai->first);
      }
    }
    if (ai != ae || bi != be) {
      return false;
    }
  }

  return true;
}

void ExecutionState::mergeStack(ExecutionState *merged,
                                std::vector<ExecutionState *> &states,
                                std::vector<ref<Expr>> &suffixes,
                                LoopHandler *loopHandler,
                                bool isEncodedWithABV,
                                PatternMatch &pm,
                                bool &usedAuxVariables) {
  usedAuxVariables = false;

  for (unsigned i = 0; i < merged->stack.size(); i++) {
    StackFrame &sf = merged->stack[i];
    auto result = LivenessAnalysis::analyzeCached(sf.kf->function);

    for (unsigned reg = 0; reg < sf.kf->numRegisters; reg++) {
      ref<Expr> &v = sf.locals[reg].value;
      bool ignore = false;
      for (ExecutionState *es : states) {
        ref<Expr> v = es->stack[i].locals[reg].value;
        if (v.isNull()) {
          ignore = true;
          break;
        }
      }
      if (ignore) {
        /* for consistency */
        v = nullptr;
        continue;
      }

      std::vector<ref<Expr>> values;
      /* TODO: update only if needed */
      State2Value valuesMap;
      for (ExecutionState *es : states) {
        ref<Expr> e = es->stack[i].locals[reg].value;
        values.push_back(e);
        valuesMap[es->getID()] = e;
      }

      if (OptimizeITEUsingExecTree && loopHandler->canUseExecTree) {
        v = mergeValuesUsingExecTree(valuesMap, loopHandler);
      } else {
        v = mergeValues(suffixes, values);
      }

      if (isEncodedWithABV && isa<SelectExpr>(v)) {
        ref<Expr> e = mergeValuesUsingPattern(valuesMap,
                                              loopHandler,
                                              pm,
                                              merged->getMergeID());
        if (!e.isNull()) {
          v = e;
          if (!usedAuxVariables && isLiveRegAt(result, sf.kf, merged->pc, reg)) {
            usedAuxVariables = true;
          }
        }
      }
      stats::mergedValuesSize += v->size;
    }
  }
}

void ExecutionState::mergeHeap(ExecutionState *merged,
                               std::vector<ExecutionState *> &states,
                               std::vector<ref<Expr>> &suffixes,
                               std::set<const MemoryObject*> &mutated,
                               LoopHandler *loopHandler,
                               bool isEncodedWithABV,
                               PatternMatch &pm,
                               bool &usedAuxVariables) {
  usedAuxVariables = false;

  for (const MemoryObject *mo : mutated) {
    const ObjectState *os = merged->addressSpace.findObject(mo);
    assert(os && !os->readOnly && "objects mutated but not writable in merging state");
    ObjectState *wos = merged->addressSpace.getWriteable(mo, os);

    /* TODO: more like knownInvalidOffset */
    std::vector<unsigned> minInvalidOffset(states.size());
    for (unsigned j = 0; j < minInvalidOffset.size(); j++) {
      minInvalidOffset[j] = mo->capacity;
    }

    std::vector<ref<Expr>> toWrite;
    for (unsigned i = 0; i < mo->capacity; i++) {
      std::vector<ref<Expr>> values;
      std::vector<ref<Expr>> neededSuffixes;
      /* TODO: update only if needed */
      State2Value valuesMap;

      for (unsigned j = 0; j < states.size(); j++) {
        ExecutionState *es = states[j];
        const ObjectState *other = es->addressSpace.findObject(mo);
        assert(other);
        assert(wos->getObject()->capacity == other->getObject()->capacity);

        /* don't track this read, otherwise the optimization is useless */
        ref<Expr> e = other->read8(i, false);
        if (!mo->hasFixedSize() && shouldOptimizeArrayValues()) {
          if (OptimizeArrayValuesByTracking) {
            if (i < other->getActualBound()) {
              values.push_back(e);
              neededSuffixes.push_back(suffixes[j]);
              valuesMap[es->getID()] = e;
            } else {
              /* the offset may be still valid */
              if (i < minInvalidOffset[j]) {
                if (es->isValidOffset(loopHandler->solver, mo, i)) {
                  values.push_back(e);
                  neededSuffixes.push_back(suffixes[j]);
                  valuesMap[es->getID()] = e;
                } else {
                  minInvalidOffset[j] = i;
                }
              }
            }
          } else if (OptimizeArrayValuesUsingSolver) {
            if (i < minInvalidOffset[j]) {
              if (es->isValidOffset(loopHandler->solver, mo, i)) {
                values.push_back(e);
                neededSuffixes.push_back(suffixes[j]);
                valuesMap[es->getID()] = e;
              } else {
                minInvalidOffset[j] = i;
              }
            }
          }
        } else {
          values.push_back(e);
          neededSuffixes.push_back(suffixes[j]);
          valuesMap[es->getID()] = e;
        }
      }

      if (!values.empty()) {
        ref<Expr> v;
        if (OptimizeArrayITEUsingExecTree && loopHandler->canUseExecTree) {
          v = mergeValuesUsingExecTree(valuesMap, loopHandler);
        } else {
          v = mergeValues(neededSuffixes, values);
        }

        if (isEncodedWithABV && isa<SelectExpr>(v)) {
          ref<Expr> e = mergeValuesUsingPattern(valuesMap,
                                                loopHandler,
                                                pm,
                                                merged->getMergeID());
          if (!e.isNull()) {
            v = e;
            usedAuxVariables = true;
          }
        }

        toWrite.push_back(v);
        stats::mergedValuesSize += v->size;
      } else {
        toWrite.push_back(nullptr);
      }
    }

    assert(toWrite.size() == mo->capacity);
    for (unsigned i = 0; i < toWrite.size(); i++) {
      ref<Expr> v = toWrite[i];
      if (!v.isNull()) {
        wos->write(i, v);
      }
    }

    if (OptimizeArrayValuesByTracking) {
      wos->setActualBound(0);
    }
  }
}

ref<Expr> ExecutionState::mergeValues(std::vector<ref<Expr>> &suffixes,
                                      std::vector<ref<Expr>> &values) {
  assert(!values.empty() && suffixes.size() == values.size());

  ref<Expr> summary = values[values.size() - 1];
  for (unsigned j = 1; j < values.size(); j++) {
    ref<Expr> cond = suffixes[values.size() - j - 1];
    ref<Expr> e = values[values.size() - j - 1];
    summary = SelectExpr::create(cond, e, summary);
  }

  return summary;
}

ref<Expr> ExecutionState::mergeValuesUsingExecTree(State2Value &valuesMap,
                                                   LoopHandler *loopHandler) {
  ExecTreeNode *n = loopHandler->tree.root;
  MergedValue v = mergeValuesFromNode(n, valuesMap);
  return v.value;
}

ExecutionState::MergedValue ExecutionState::mergeValuesFromNode(ExecTreeNode *n,
                                                                State2Value &valuesMap) {
  if (!n) {
    /* if the node is missing, then we have an incomplete subtree */
    return MergedValue(nullptr, false, nullptr);
  }

  ref<Expr> trueExpr = ConstantExpr::create(1, Expr::Bool);
  if (n->isLeaf()) {
    auto i = valuesMap.find(n->stateID);
    /* a joined node corresponds to an incomplete subtree */
    bool isComplete = !n->isJoined;
    if (i == valuesMap.end()) {
      return MergedValue(nullptr, isComplete, nullptr);
    } else {
      return MergedValue(i->second, isComplete, trueExpr);
    }
  }

  /* TODO: left/right or right/left? */
  MergedValue lv = mergeValuesFromNode(n->left, valuesMap);
  MergedValue rv = mergeValuesFromNode(n->right, valuesMap);

  bool isComplete = lv.isComplete && rv.isComplete;

  /* TODO: order of conditions? */
  if (lv.value.isNull()) {
    if (rv.value.isNull()) {
      return MergedValue(nullptr, isComplete, nullptr);
    } else {
      return MergedValue(
        rv.value,
        isComplete,
        isComplete ? trueExpr : AndExpr::create(n->right->e, rv.guard)
      );
    }
  } else {
    if (rv.value.isNull()) {
      return MergedValue(
        lv.value,
        isComplete,
        isComplete ? trueExpr : AndExpr::create(n->left->e, lv.guard)
      );
    } else {
      ref<Expr> guard;
      if (isComplete) {
        guard = trueExpr;
      } else {
        /* TODO: can avoid n->right->e? */
        guard = OrExpr::create(
          AndExpr::create(n->left->e, lv.guard),
          AndExpr::create(n->right->e, rv.guard)
        );
      }

      ref<Expr> ite = SelectExpr::create(
        AndExpr::create(n->left->e, lv.guard),
        lv.value,
        rv.value
      );
      if (SimplifyITE) {
        ite = simplifyITE(ite);
      }
      return MergedValue(
        ite,
        isComplete,
        guard
      );
    }
  }
}

ref<Expr> ExecutionState::mergeValuesUsingPattern(State2Value &valuesMap,
                                                  LoopHandler *loopHandler,
                                                  PatternMatch &pm,
                                                  std::uint32_t mergeID) {
  ParametrizedExpr solution;
  if (!generateMergedValue(pm,
                           loopHandler->tree,
                           valuesMap,
                           mergeID,
                           *loopHandler->solver,
                           solution)) {
    return nullptr;
  }

  return solution.e;
}

bool ExecutionState::areEquiv(TimingSolver *solver,
                              const ExecutionState *sa,
                              const ExecutionState *sb,
                              bool checkPCEquivalence) {
  ConstraintSet empty;

  ref<Expr> pcA = ConstantExpr::create(1, Expr::Bool);
  for (ref<Expr> e : sa->constraints) {
    pcA = AndExpr::create(pcA, e);
  }

  ref<Expr> pcB = ConstantExpr::create(1, Expr::Bool);
  for (ref<Expr> e : sb->constraints) {
    pcB = AndExpr::create(pcB, e);
  }

  ref<Expr> pcEquiv;
  if (checkPCEquivalence) {
    pcEquiv = AndExpr::create(
      OrExpr::create(NotExpr::create(pcA), pcB),
      OrExpr::create(NotExpr::create(pcB), pcA)
    );
  } else {
    /* make sure that A is consistent (satisfiable) */
    bool isSatisfiable = false;
    SolverQueryMetaData meta;
    assert(solver->mayBeTrue(nullptr, empty, pcA, isSatisfiable, meta));
    if (!isSatisfiable) {
      return false;
    }
    /* A --> B */
    pcEquiv = OrExpr::create(NotExpr::create(pcA), pcB);
  }

  bool isTrue = false;
  SolverQueryMetaData meta;
  assert(solver->mustBeTrue(nullptr, empty, pcEquiv, isTrue, meta));
  if (!isTrue) {
    return false;
  }

  for (unsigned i = 0; i < sa->stack.size(); i++) {
    const StackFrame &sf = sa->stack[i];
    for (unsigned reg = 0; reg < sf.kf->numRegisters; reg++) {
      ref<Expr> v1 = sa->stack[i].locals[reg].value;
      ref<Expr> v2 = sb->stack[i].locals[reg].value;
      if (v1.isNull() || v2.isNull()) {
        assert(v1.isNull() && v2.isNull());
        continue;
      }

      bool isEqual;
      SolverQueryMetaData meta;
      assert(solver->mustBeTrue(nullptr,
                                sa->constraints,
                                EqExpr::create(v1, v2),
                                isEqual,
                                meta));
      if (!isEqual) {
        return false;
      }
    }
  }

  for (auto i : sa->addressSpace.objects) {
    const MemoryObject *mo = i.first;
    ref<ObjectState> os = i.second;
    const ObjectState *otherOS = sb->addressSpace.findObject(mo);
    for (unsigned i = 0; i < mo->capacity; i++) {
      bool inRange = false;
      SolverQueryMetaData meta;
      ref<Expr> rangeCond = UltExpr::create(ConstantExpr::create(i, Expr::Int64), mo->getSizeExpr());
      assert(solver->mayBeTrue(nullptr,
                               sa->constraints,
                               rangeCond,
                               inRange,
                               meta));
      if (!inRange) {
        continue;
      }

      ref<Expr> v1 = os->read8(i, false);
      ref<Expr> v2 = otherOS->read8(i, false);

      bool isEqual;
      ConstraintSet tmp(sa->constraints);
      tmp.push_back(rangeCond);
      assert(solver->mustBeTrue(nullptr,
                                tmp,
                                EqExpr::create(v1, v2),
                                isEqual,
                                meta));
      if (!isEqual) {
        return false;
      }
    }
  }

  return true;
}

bool ExecutionState::shouldOptimizeArrayValues() {
  return OptimizeArrayValuesByTracking ||
         OptimizeArrayValuesUsingSolver ||
         OptimizeArrayValuesUsingITERewrite;
}

void ExecutionState::optimizeArrayValues(std::set<const MemoryObject*> mutated,
                                         TimingSolver *solver) {
  for (const MemoryObject *mo : mutated) {
    /* TODO: create only if needed */
    const ObjectState *os = addressSpace.findObject(mo);
    ObjectState *wos = addressSpace.getWriteable(mo, os);

    for (unsigned i = 0; i < mo->capacity; i++) {
      ref<Expr> v = wos->read8(i);
      if (isa<ConstantExpr>(v)) {
        continue;
      }

      if (!isValidOffset(solver, mo, i)) {
        /* no need to check further... */
        continue;
      }

      ref<Expr> x = simplifyArrayElement(mo, i, v, solver);
      wos->write(i, x);
    }
  }
}

ref<Expr> ExecutionState::simplifyArrayElement(const MemoryObject *mo,
                                               uint64_t offset,
                                               ref<Expr> v,
                                               TimingSolver *solver) {
  if (isa<ConstantExpr>(v) || mo->hasFixedSize()) {
    return v;
  }

  bool revisit = false;
  do {
    ITEOptimizer visitor(*this, offset, mo->getSizeExpr(), solver);
    ref<Expr> e = visitor.visit(v);
    revisit = !isa<ConstantExpr>(e) && visitor.changed;
    v = e;
  } while (revisit);

  return v;
}

ref<Expr> ExecutionState::mergeConstraints(std::vector<ExecutionState *> &states) {
  ref<Expr> orExpr = ConstantExpr::create(0, Expr::Bool);
  for (ExecutionState *es : states) {
    /* build suffix conjunction */
    ref<Expr> andExpr = ConstantExpr::create(1, Expr::Bool);
    for (ref<Expr> e : es->suffixConstraints) {
      andExpr = AndExpr::create(andExpr, e);
    }

    /* update disjunction */
    orExpr = OrExpr::create(orExpr, andExpr);
  }

  return orExpr;
}

ref<Expr> ExecutionState::mergeConstraintsWithExecTree(LoopHandler *loopHandler,
                                                       std::vector<ExecutionState *> &states) {
  std::set<uint32_t> ids;
  for (ExecutionState *es : states) {
    ids.insert(es->getID());
  }

  ExecTreeNode *n = loopHandler->tree.root;
  auto p = mergeConstraintsFromNode(n, ids);
  return p.constraint;
}

/*
 * we can use the solver to reduce 'and' expression to rv/lv (in some cases),
 * but it doesn't seem to be that helpful in practice...
 */
ExecutionState::MergedConstraint ExecutionState::mergeConstraintsFromNode(ExecTreeNode *n,
                                                                          std::set<uint32_t> &ids) {
  if (!n) {
    return MergedConstraint(nullptr, false);
  }

  if (n->isLeaf()) {
    auto i = ids.find(n->stateID);
    if (i == ids.end()) {
      return MergedConstraint(nullptr, false);
    } else {
      /* a joined node corresponds to an incomplete subtree */
      return MergedConstraint(n->e, !n->isJoined);
    }
  }

  /* TODO: left/right or right/left? */
  MergedConstraint lp = mergeConstraintsFromNode(n->left, ids);
  MergedConstraint rp = mergeConstraintsFromNode(n->right, ids);

  ref<Expr> lv = lp.constraint;
  ref<Expr> rv = rp.constraint;

  if (lv.isNull()) {
    if (rv.isNull()) {
      return MergedConstraint(nullptr, false);
    } else {
      return MergedConstraint(AndExpr::create(n->e, rv), false);
    }
  } else {
    if (rv.isNull()) {
      return MergedConstraint(AndExpr::create(n->e, lv), false);
    } else {
      if (lp.isFull && rp.isFull) {
        return MergedConstraint(n->e, true);
      } else {
        /* TODO: encode as or(and, and)? */
        return MergedConstraint(AndExpr::create(n->e, OrExpr::create(lv, rv)), false);
      }
    }
  }
}

bool ExecutionState::isValidOffset(TimingSolver *solver,
                                   const MemoryObject *mo,
                                   uint64_t offset) {
  Solver::Validity result;
  ref<Expr> range = UltExpr::create(ConstantExpr::create(offset, Expr::Int64), mo->getSizeExpr());
  bool success = solver->evaluate(this, constraints, range, result, queryMetaData, true);
  assert(success);

  return result != Solver::False;
}

bool ExecutionState::isLiveRegAt(const LivenessAnalysis::Result &result,
                                 KFunction *kf,
                                 KInstruction *kinst,
                                 unsigned reg) {
  auto i = kf->inverseRegisterMap.find(reg);
  if (i == kf->inverseRegisterMap.end()) {
    /* TODO: why? */
    return true;
  }

  Value *regValue = i->second;
  return LivenessAnalysis::isLiveAt(result, kinst->inst, regValue, nullptr);
}

bool ExecutionState::compareStack(ExecutionState &s1,
                                  ExecutionState &s2) {
  StackFrame &sf1 = s1.stack.back();
  StackFrame &sf2 = s2.stack.back();

  /* identical in both states */
  KInstruction *kinst = s1.pc;
  KFunction *kf = sf1.kf;

  auto result = LivenessAnalysis::analyzeCached(kf->function);

  for (unsigned reg = 0; reg < kf->numRegisters; reg++) {
    if (!isLiveRegAt(result, kf, kinst, reg)) {
      continue;
    }

    ref<Expr> v1 = sf1.locals[reg].value;
    ref<Expr> v2 = sf2.locals[reg].value;
    if (v1.isNull() || v2.isNull()) {
      if (!(v1.isNull() && v2.isNull())) {
        return false;
      }
    } else {
      if (*v1 != *v2) {
        return false;
      }
    }
  }

  return true;
}

bool ExecutionState::compareHeap(ExecutionState &s1,
                                 ExecutionState &s2,
                                 std::set<const MemoryObject *> &mutated) {
  for (const MemoryObject *mo : mutated) {
    const ObjectState *os1 = s1.addressSpace.findObject(mo);
    const ObjectState *os2 = s2.addressSpace.findObject(mo);
    for (unsigned i = 0; i < mo->capacity; i++) {
      ref<Expr> e1 = os1->read8(i, false);
      ref<Expr> e2 = os2->read8(i, false);
      if (*e1 != *e2) {
        return false;
      }
    }
  }

  return true;
}

std::uint32_t ExecutionState::getMergeID() const {
  return UseLocalMergeID ? localMergeID : mergeID;
}

void ExecutionState::incMergeID() {
  mergeID++;
  localMergeID++;
}

void ExecutionState::addAuxArray() {
  /* TODO: remove the size argument? */
  const Array *array = getArrayForAuxVariable(
    getMergeID(),
    QuantifiedExpr::AUX_VARIABLE_WIDTH / 8
  );
  renamingMap[array] = nullptr;
  mapAuxArrays();
}

void ExecutionState::mapAuxArrays() {
  unsigned index = 0;
  for (auto i : renamingMap) {
    const Array *array = i.first;
    renamingMap[array] = cloneArray(array, index);
    index++;
  }
}

void ExecutionState::rewriteConstraints() {
  rewrittenConstraints.clear();
  for (ref<Expr> e : constraints) {
    ref<Expr> renamed = rename(e, renamingMap);
    rewrittenConstraints.push_back(renamed);
  }
}
