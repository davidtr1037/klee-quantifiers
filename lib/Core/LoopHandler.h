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

  /* TODO: make signed? */
  int activeStates;

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

  bool isLiveAt(LivenessAnalysis::Result &result,
                KFunction *kf,
                KInstruction *kinst,
                unsigned reg);

  bool compareStack(ExecutionState &s1, ExecutionState &s2);

  bool compareHeap(ExecutionState &s1,
                   ExecutionState &s2,
                   std::set<const MemoryObject *> &mutated);

  bool shouldMerge(ExecutionState &s1, ExecutionState &s2);

  void discardState(ExecutionState *es);

  void removeSubTree(ExecTreeNode *src);

  void mergeNodes(ExecTreeNode *n1, ExecTreeNode *n2);

  bool mergeIntermediateState(ExecTreeNode *target);

  void mergeIntermediateStates();

  bool validateMerge(std::vector<ExecutionState *> &states,
                     ExecutionState *merged);

  LivenessAnalysis::Result getLivenessAnalysisResult(llvm::Function *f);

  class ReferenceCounter _refCount;

  Executor *executor;

  TimingSolver *solver;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;

  ExecTree tree;

  bool canUseExecTree;

  std::map<llvm::Function *, LivenessAnalysis::Result> cache;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
