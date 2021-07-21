#include <list>
#include <set>
#include <map>

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/InstIterator.h>
#include <llvm/IR/CFG.h>
#include <llvm/Support/raw_ostream.h>

#include <klee/Support/ErrorHandling.h>

#include "LivenessAnalysis.h"


using namespace llvm;
using namespace std;

namespace klee {

void LivenessAnalysis::analyze(Function *f,
                               LiveSet &liveIn,
                               LiveSet &liveOut) {
  bool changed;
  do {
    changed = runIteration(f, liveIn, liveOut);
  } while (changed);
}

bool LivenessAnalysis::runIteration(Function *f,
                                    LiveSet &liveIn,
                                    LiveSet &liveOut) {
  list<Instruction *> worklist;
  set<Instruction *> visited;
  bool changed = false;

  for (BasicBlock &bb : *f) {
    for (Instruction &inst : bb) {
      if (isa<ReturnInst>(&inst)) {
        worklist.push_back(&inst);
      }
    }
  }

  while (!worklist.empty()) {
    Instruction *inst = worklist.front();
    worklist.pop_front();

    if (visited.find(inst) != visited.end()) {
      continue;
    }

    Instruction *prev = inst->getPrevNode();
    if (prev) {
      worklist.push_back(prev);
    } else {
      for (BasicBlock *pred : predecessors(inst->getParent())) {
        Instruction *last = pred->getTerminator();
        worklist.push_back(last);
      }
    }

    if (isa<CallInst>(inst)) {
      CallInst *callInst = dyn_cast<CallInst>(inst);
      Function *f = callInst->getCalledFunction();
      if (f) {
        if (f->isIntrinsic()) {
          klee_warning_once(0, "ignoring function: %s", f->getName().data());
          continue;
        }
      }
    }

    set<Value *> toKill, toGen;
    gen(inst, toGen);
    kill(inst, toKill);

    /* live-out */
    std::vector<Instruction *> successors;
    if (isa<TerminatorInst>(inst)) {
      TerminatorInst *termInst = dyn_cast<TerminatorInst>(inst);
      for (BasicBlock *bb : termInst->successors()) {
        successors.push_back(&bb->front());
      }
    } else {
      successors.push_back(inst->getNextNode());
    }
    for (Instruction *s : successors) {
      for (Value *v : liveIn[s]) {
        auto r = liveOut[inst].insert(v);
        if (r.second) {
          changed = true;
        }
      }
    }

    /* live-in */
    for (Value *v : toGen) {
      auto r = liveIn[inst].insert(v);
      if (r.second) {
        changed = true;
      }
    }
    for (Value *v : liveOut[inst]) {
      if (toKill.find(v) == toKill.end()) {
        auto r = liveIn[inst].insert(v);
        if (r.second) {
          changed = true;
        }
      }
    }

    visited.insert(inst);
  }

  return changed;
}

void LivenessAnalysis::gen(Instruction *inst,
                           set<Value *> &variables) {
  for (unsigned i = 0; i < inst->getNumOperands(); i++) {
    Value *v = inst->getOperand(i);
    if (isa<Instruction>(v) || isa<Argument>(v)) {
      variables.insert(v);
    }
  }
}

void LivenessAnalysis::kill(Instruction *inst,
                            set<Value *> &variables) {
  if (inst->getType()->isVoidTy()) {
    return;
  }

  Value *v = (Value *)(inst);
  variables.insert(v);
}

void LivenessAnalysis::dumpLiveSet(LiveSet &ls) {
  errs() << "--- live set ---\n";
  for (auto i : ls) {
    errs() << "instruction: " << *i.first << "\n";
    for (Value *v : i.second) {
      errs() << "-- live: " << *v << "\n";
    }
  }
}

}
