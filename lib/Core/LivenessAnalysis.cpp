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
      if (isa<ReturnInst>(&inst) || isa<UnreachableInst>(&inst)) {
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

    set<GuardedValue> toKill, toGen;
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
      for (const GuardedValue &v : liveIn[s]) {
        if (updateOutSet(inst, s, liveOut, v)) {
          changed = true;
        }
      }
    }

    /* live-in */
    for (const GuardedValue &v : toGen) {
      auto r = liveIn[inst].insert(v);
      if (r.second) {
        changed = true;
      }
    }
    for (const GuardedValue &v : liveOut[inst]) {
      bool found = false;
      for (const GuardedValue &w : toKill) {
        if (w.value == v.value) {
          found = true;
          break;
        }
      }

      if (!found) {
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

bool LivenessAnalysis::shouldIgnore(llvm::Instruction *inst) {
  if (isa<CallInst>(inst)) {
    CallInst *callInst = dyn_cast<CallInst>(inst);
    Function *f = callInst->getCalledFunction();
    if (f) {
      if (f->isIntrinsic()) {
        klee_warning_once(0, "ignoring function: %s", f->getName().data());
        return true;
      }
    }
  }

  return false;
}

void LivenessAnalysis::gen(Instruction *inst,
                           set<GuardedValue> &variables) {
  if (shouldIgnore(inst)) {
    return;
  }

  if (isa<PHINode>(inst)) {
    PHINode *phi = dyn_cast<PHINode>(inst);
    for (unsigned i = 0; i < phi->getNumIncomingValues(); i++) {
      Value *v = phi->getIncomingValue(i);
      if (isa<Instruction>(v) || isa<Argument>(v)) {
        BasicBlock *bb = phi->getIncomingBlock(i);
        variables.insert(GuardedValue(v, bb));
      }
    }
    return;
  }

  for (unsigned i = 0; i < inst->getNumOperands(); i++) {
    Value *v = inst->getOperand(i);
    if (isa<Instruction>(v) || isa<Argument>(v)) {
      variables.insert(GuardedValue(v, nullptr));
    }
  }
}

void LivenessAnalysis::kill(Instruction *inst,
                            set<GuardedValue> &variables) {
  if (inst->getType()->isVoidTy()) {
    return;
  }
  if (shouldIgnore(inst)) {
    return;
  }

  Value *v = (Value *)(inst);
  variables.insert(GuardedValue(v, nullptr));
}

bool LivenessAnalysis::updateOutSet(Instruction *inst,
                                    Instruction *successor,
                                    LiveSet &liveOut,
                                    const GuardedValue &v) {
  /* TODO: a better solution? */
  if (v.bb && inst->getParent() != successor->getParent()) {
    if (v.bb == inst->getParent()) {
      auto r = liveOut[inst].insert(GuardedValue(v.value, nullptr));
      return r.second;
    } else {
      return false;
    }
  }
  auto r = liveOut[inst].insert(v);
  return r.second;
}

bool LivenessAnalysis::isLiveAt(const Result &result,
                                Instruction *at,
                                Value *value,
                                BasicBlock *src) {
  auto i = result.liveIn.find(at);
  if (i == result.liveIn.end()) {
    assert(false);
  }

  const std::set<GuardedValue> &variables = i->second;
  for (const GuardedValue &v : variables) {
    if (v.value == value) {
      return src ? v.bb == src : true;
    }
  }

  return false;
}

void LivenessAnalysis::dumpLiveSet(LiveSet &ls) {
  errs() << "--- live set ---\n";
  for (auto i : ls) {
    errs() << "instruction: " << *i.first << "\n";
    for (const GuardedValue &v : i.second) {
      errs() << "-- live: " << *v.value << "\n";
    }
  }
}

const LivenessAnalysis::Result &LivenessAnalysis::analyzeCached(Function *f) {
  static std::map<llvm::Function *, Result> cache;
  auto i = cache.find(f);
  if (i != cache.end()) {
    return i->second;
  }

  Result result;
  analyze(f, result.liveIn, result.liveOut);
  auto p = cache.insert(std::make_pair(f, result));
  return p.first->second;
}

}
