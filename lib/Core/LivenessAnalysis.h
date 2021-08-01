#ifndef LIVENESS_ANALYSIS_H
#define LIVENESS_ANALYSIS_H

#include <set>
#include <map>

#include <llvm/IR/Value.h>
#include <llvm/IR/Instruction.h>
#include <llvm/IR/Function.h>

namespace klee {

class LivenessAnalysis {

public:

  typedef std::map<llvm::Instruction *, std::set<llvm::Value *>> LiveSet;

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
                  std::set<llvm::Value *> &variables);

  static void kill(llvm::Instruction *inst,
                   std::set<llvm::Value *> &variables);

  static void dumpLiveSet(LiveSet &ls);

};

}

#endif
