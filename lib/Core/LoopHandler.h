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
#include <unordered_map>
#include <stdint.h>
#include <vector>

namespace llvm {
class Instruction;
}

namespace klee {

extern llvm::cl::opt<bool> UseLoopMerge;
extern llvm::cl::opt<bool> StartLoopMergeOnBranch;

class Executor;
class ExecutionState;

struct LoopExit {
  llvm::Instruction *inst;
  uint32_t incomingBBIndex;

  LoopExit(llvm::Instruction *inst, uint32_t incomingBBIndex) :
    inst(inst), incomingBBIndex(incomingBBIndex) {

  }

  bool operator==(const LoopExit &other) const;
};

struct LoopExitHash {
  std::size_t operator() (const LoopExit &loopExit) const {
    std::size_t h = std::hash<llvm::Instruction *>()(loopExit.inst);
    return h;
  }
};

class LoopHandler {

private:

  /* TODO: rename */
  typedef std::vector<ExecutionState *> StateSet;

  struct MergeSubGroupInfo {
    StateSet states;
    std::vector<PatternMatch> matches;

    MergeSubGroupInfo(const StateSet &states,
                      const std::vector<PatternMatch> &matches) :
      states(states),
      matches(matches) {

    }

    MergeSubGroupInfo(const StateSet &states) :
      states(states) {

    }

  };

  struct MergeGroupInfo {
    std::vector<MergeSubGroupInfo> subGroups;

    MergeGroupInfo(const std::vector<MergeSubGroupInfo> &subGroups) :
      subGroups(subGroups) {

    }

    size_t getStatesCount() const {
      size_t total = 0;
      for (const MergeSubGroupInfo &info : subGroups) {
        total += info.states.size();
      }
      return total;
    }
  };

  unsigned closedStateCount;

  std::vector<ExecutionState *> openStates;

  std::unordered_map<LoopExit, StateSet, LoopExitHash> mergeGroupsByExit;

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

  void discardIntermediateState(ExecutionState *es);

  void addClosedState(ExecutionState *es, llvm::Instruction *mp);

  bool shouldForceCFGBasedMerging();

  bool extractPatterns(const std::set<uint32_t> &ids,
                       std::vector<PatternMatch> &matches);

  bool shouldUsePatternBasedMerging(std::vector<PatternMatch> &matches,
                                    std::vector<StateSet> &matchedStates);

  void splitStatesByCFG(std::vector<MergeGroupInfo> &result);

  void splitStatesByPattern(std::vector<MergeGroupInfo> &result);

  void splitStates(std::vector<MergeGroupInfo> &result);

  ExecutionState *mergeSubGroup(MergeSubGroupInfo &info,
                                bool isComplete);

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
