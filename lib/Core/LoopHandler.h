#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "TimingSolver.h"
#include "Memory.h"
#include "ExecTree.h"
#include "LivenessAnalysis.h"

#include "klee/ADT/Ref.h"
#include "klee/Expr/Constraints.h"
#include "klee/Module/KModule.h"

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

  /* TODO: signed or unsigned? */
  int activeStates;

  unsigned earlyTerminated;

public:

  LoopHandler(Executor *executor, ExecutionState *es, llvm::Loop *loop);

  ~LoopHandler();

  void addOpenState(ExecutionState *es);

  void removeOpenState(ExecutionState *es);

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  void splitStates(std::vector<MergeGroup> &result);

  void releaseStates();

  void markEarlyTerminated(ExecutionState &state);

  unsigned getEarlyTerminated();

  bool shouldMerge(ExecutionState &s1, ExecutionState &s2);

  void discardState(ExecutionState *es);

  void discardSubTree(ExecTreeNode *src,
                      ExecTreeNode *ancestor);

  void mergeNodes(ExecTreeNode *n1,
                  ExecTreeNode *n2,
                  ExecutionState *s1,
                  ExecutionState *s2);

  bool mergeNodes(ExecTreeNode *n1, ExecTreeNode *n2);

  bool mergeIntermediateState(ExecTreeNode *target);

  bool mergeIntermediateStates();

  bool joinIntermediateStates();

  bool transform();

  bool validateMerge(std::vector<ExecutionState *> &states,
                     ExecutionState *merged);

  class ReferenceCounter _refCount;

  Executor *executor;

  TimingSolver *solver;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;

  ExecTree tree;

  bool canUseExecTree;

  bool shouldTransform;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
