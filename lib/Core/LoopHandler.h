#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "TimingSolver.h"
#include "ExecTree.h"

#include "klee/ADT/Ref.h"
#include "klee/Expr/Constraints.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Analysis/LoopInfo.h"

#include <map>
#include <stdint.h>
#include <vector>

namespace llvm {
class Instruction;
}

namespace klee {

extern llvm::cl::opt<bool> UseLoopMerge;
extern llvm::cl::opt<bool> DebugLoopHandler;
extern llvm::cl::opt<bool> UseOptimizedMerge;

class Executor;
class ExecutionState;

class LoopHandler {

private:

  typedef std::vector<ExecutionState *> MergeGroup;

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::map<llvm::Instruction *, MergeGroup> mergeGroupsByExit;

  unsigned activeStates;

  unsigned earlyTerminated;

public:

  LoopHandler(Executor *executor, ExecutionState *es, llvm::Loop *loop);

  ~LoopHandler();

  void addOpenState(ExecutionState *es);

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  void removeOpenState(ExecutionState *es);

  void splitStates(std::vector<MergeGroup> &result);

  void releaseStates();

  void markEarlyTerminated(ExecutionState &state);

  unsigned getEarlyTerminated();

  bool validateMerge(std::vector<ExecutionState *> &states,
                     ExecutionState *merged);

  class ReferenceCounter _refCount;

  Executor *executor;

  TimingSolver *solver;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;

  ExecTree tree;

  bool canUseExecTree;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
