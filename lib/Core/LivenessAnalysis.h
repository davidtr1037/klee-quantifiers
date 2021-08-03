#ifndef LIVENESS_ANALYSIS_H
#define LIVENESS_ANALYSIS_H

#include <set>
#include <map>
#include <tuple>

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

namespace klee {

struct GuardedValue {
  llvm::Value *v;
  llvm::BasicBlock *bb;

  GuardedValue() : v(nullptr), bb(nullptr) {

  }

  GuardedValue(llvm::Value *v, llvm::BasicBlock *bb) : v(v), bb(bb) {

  }

  bool operator==(const GuardedValue& other) const {
    return v == other.v && bb == other.bb;
  }

  bool operator<(const GuardedValue& other) const {
    return std::tie(v, bb) < std::tie(other.v, other.bb);
  }

};

class LivenessAnalysis {

public:

  typedef std::map<llvm::Instruction *, std::set<GuardedValue>> LiveSet;

  struct Result {
      LiveSet liveIn;
      LiveSet liveOut;
  };

  static void analyze(llvm::Function *f,
                      LiveSet &liveIn,
                      LiveSet &liveOut);

  static bool runIteration(llvm::Function *f,
                           LiveSet &liveIn,
                           LiveSet &liveOut);

  static bool shouldIgnore(llvm::Instruction *inst);

  static void gen(llvm::Instruction *inst,
                  std::set<GuardedValue> &variables);

  static void kill(llvm::Instruction *inst,
                   std::set<GuardedValue> &variables);

  static bool updateOutSet(llvm::Instruction *inst,
                           llvm::Instruction *successor,
                           LiveSet &liveOut,
                           const GuardedValue &v);

  static void dumpLiveSet(LiveSet &ls);

};

}

#endif
