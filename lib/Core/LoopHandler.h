#ifndef KLEE_LOOPHANDLER_H
#define KLEE_LOOPHANDLER_H

#include "TimingSolver.h"
#include "Memory.h"
#include "ExecTree.h"
#include "LivenessAnalysis.h"
#include "PatternExtraction.h"

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
extern llvm::cl::opt<bool> UseOptimizedMerge;

class Executor;
class ExecutionState;

class LoopHandler {

private:

  typedef std::vector<ExecutionState *> MergeGroup;

  struct MergeGroupInfo {
    MergeGroup states;
    std::vector<PatternMatch> matches;

    MergeGroupInfo(const MergeGroup &states,
                   const std::vector<PatternMatch> &matches) :
      states(states),
      matches(matches) {

    }

    MergeGroupInfo(const MergeGroup &states) :
      states(states) {

    }

  };

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::map<llvm::Instruction *, MergeGroup> mergeGroupsByExit;

  /* TODO: signed or unsigned? */
  int activeStates;

  unsigned earlyTerminated;

public:

  LoopHandler(Executor *executor,
              ExecutionState *es,
              llvm::Loop *loop,
              bool useIncrementalMergingSearch = true);

  ~LoopHandler();

  void addInitialState(ExecutionState *es);

  void addOpenState(ExecutionState *es);

  void removeOpenState(ExecutionState *es);

  void pauseOpenState(ExecutionState *es);

  void resumeClosedState(ExecutionState *es);

  void discardOpenState(ExecutionState *es, const char *reason);

  void discardClosedState(ExecutionState *es,
                          const char *reason,
                          bool isFullyExplored);

  void discardState(ExecutionState *es,
                    const char *reason,
                    bool isFullyExplored);

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  void splitStates(std::vector<MergeGroupInfo> &result);

  ExecutionState *merge(MergeGroup &states,
                        bool isComplete,
                        std::vector<PatternMatch> &matches);

  ExecutionState *mergeGroup(MergeGroupInfo &groupInfo, bool isComplete);

  void releaseStates();

  void markEarlyTerminated(ExecutionState &state);

  unsigned getEarlyTerminated();

  bool canMergeNodes(ExecTreeNode *n1, ExecTreeNode *n2);

  bool shouldMerge(ExecutionState &s1, ExecutionState &s2);

  bool discardStateByID(unsigned id);

  void discardSubTree(ExecTreeNode *src,
                      ExecTreeNode *ancestor);

  void setSuffixConstraints(ExecutionState *merged,
                            ExecTreeNode *ancestor,
                            ref<Expr> condition);

  void mergeNodes(ExecTreeNode *n1,
                  ExecTreeNode *n2,
                  ExecutionState *s1,
                  ExecutionState *s2);

  bool mergeNodes(ExecTreeNode *n1, ExecTreeNode *n2);

  bool mergeIntermediateState(ExecTreeNode *target);

  bool runMergeTransformationNaive();

  bool runMergeTransformation();

  bool runJoinTransformation();

  bool transform();

  bool validateMerge(std::vector<ExecutionState *> &states,
                     ExecutionState *merged);

  void dumpStats() const;

  class ReferenceCounter _refCount;

  Executor *executor;

  TimingSolver *solver;

  llvm::Loop *loop;

  ConstraintSet initialConstraints;

  ExecTree tree;

  bool canUseExecTree;

  bool shouldTransform;

  bool useIncrementalMergingSearch;

  /* statistics */
  uint64_t mergeCount;
  uint64_t joinCount;
};

}

#endif	/* KLEE_MERGEHANDLER_H */
