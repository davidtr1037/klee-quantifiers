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

    set<StringRef> toKill, toGen;
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
      for (StringRef n : liveIn[s]) {
        auto r = liveOut[inst].insert(n);
        if (r.second) {
          changed = true;
        }
      }
    }

    /* live-in */
    for (StringRef n : toGen) {
      auto r = liveIn[inst].insert(n);
      if (r.second) {
        changed = true;
      }
    }
    for (StringRef name : liveOut[inst]) {
      if (toKill.find(name) == toKill.end()) {
        auto r = liveIn[inst].insert(name);
        if (r.second) {
          changed = true;
        }
      }
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

    visited.insert(inst);
  }

  return changed;
}

void LivenessAnalysis::gen(Instruction *inst,
                           set<StringRef> &variables) {
  for (unsigned i = 0; i < inst->getNumOperands(); i++) {
    Value *v = inst->getOperand(i);
    if (v->hasName()) {
      variables.insert(v->getName());
    }
  }
}

void LivenessAnalysis::kill(Instruction *inst,
                            set<StringRef> &variables) {
  if (inst->getType()->isVoidTy()) {
    return;
  }

  Value *v = (Value *)(inst);
  if (v->hasName()) {
    variables.insert(v->getName());
  }
}

void LivenessAnalysis::dumpLiveSet(LiveSet &ls) {
  errs() << "--- live set ---\n";
  for (auto i : ls) {
    errs() << "instruction: " << *i.first << "\n";
    for (StringRef name : i.second) {
      errs() << "-- live: " << name << "\n";
    }
  }
}

}
