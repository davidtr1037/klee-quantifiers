#include "LoopHandler.h"
#include "ExecTreeIterator.h"

#include "CoreStats.h"
#include "ExecutionState.h"
#include "Executor.h"
#include "Searcher.h"
#include "Memory.h"
#include "PatternExtraction.h"

#include <list>

using namespace std;
using namespace llvm;

namespace klee {

cl::OptionCategory LoopCat("Loop merging options",
                           "These options control path merging.");

cl::opt<bool> UseLoopMerge(
    "use-loop-merge",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseOptimizedMerge(
    "use-optimized-merge",
    cl::init(true),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> ValidateMerge(
    "validate-merge",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<unsigned> MaxStatesToMerge(
    "max-states-to-merge",
    cl::init(10000),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> SplitByPattern(
    "split-by-pattern",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> SplitByCFG(
    "split-by-cfg",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));


cl::opt<bool> UseForwardExtract(
    "use-forward-extract",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseMergeTransformation(
    "use-merge-transformation",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseJoinTransformation(
    "use-join-transformation",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> DebugMergeTransformation(
    "debug-merge-transformation",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> RestrictPatternBaseMerging(
    "restrict-pattern-based-merging",
    cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<unsigned> MaxPatterns(
    "max-patterns",
    cl::init(10),
    cl::desc(""),
    cl::cat(klee::LoopCat));

LoopHandler::LoopHandler(Executor *executor,
                         ExecutionState *es,
                         Loop *loop,
                         bool useIncrementalMergingSearch)
    : closedStateCount(0),
      activeStates(0),
      earlyTerminated(0),
      executor(executor),
      solver(executor->solver),
      loop(loop),
      tree(es->getID()),
      canUseExecTree(true),
      shouldTransform(false),
      useIncrementalMergingSearch(useIncrementalMergingSearch),
      mergeCount(0),
      joinCount(0) {
  assert(loop);
  addInitialState(es);
}

LoopHandler::~LoopHandler() {
  if (executor->haltExecution) {
    /* if execution is interrupted, may contain unreleased states */
    return;
  }

  assert(activeStates == 0);
  for (const auto &i: mergeGroupsByExit) {
    const vector<ExecutionState *> &states = i.second;
    assert(states.empty());
  }
}

void LoopHandler::addInitialState(ExecutionState *es) {
  addOpenState(es);
  for (ref<Expr> e : es->constraints) {
    initialConstraints.push_back(e);
  }
  if (useIncrementalMergingSearch) {
    executor->incrementalMergingSearcher->removeState(es);
    executor->incrementalMergingSearcher->internalSearcher->addState(es);
  }
}

void LoopHandler::addOpenState(ExecutionState *es){
  openStates.push_back(es);
  activeStates++;
}

void LoopHandler::removeOpenState(ExecutionState *es) {
  auto it = find(openStates.begin(), openStates.end(), es);
  assert(it != openStates.end());
  swap(*it, openStates.back());
  openStates.pop_back();
}

void LoopHandler::pauseOpenState(ExecutionState *es) {
  if (useIncrementalMergingSearch) {
    Searcher *internalSearcher = executor->incrementalMergingSearcher->internalSearcher;
    internalSearcher->removeState(es);
  } else {
    assert(executor->mergingSearcher->inCloseMerge.find(es) == executor->mergingSearcher->inCloseMerge.end());
    executor->mergingSearcher->inCloseMerge.insert(es);
    executor->mergingSearcher->pauseState(*es);
  }
}

void LoopHandler::resumeClosedState(ExecutionState *es) {
  if (useIncrementalMergingSearch) {
    executor->incrementalMergingSearcher->addState(es);
  } else {
    executor->mergingSearcher->inCloseMerge.erase(es);
    executor->mergingSearcher->continueState(*es);
  }
}

void LoopHandler::discardOpenState(ExecutionState *es, const char *reason) {
  executor->terminateStateEarly(*es, reason);
  executor->interpreterHandler->decUnmergedExploredPaths();
}

void LoopHandler::discardClosedState(ExecutionState *es,
                                     const char *reason,
                                     bool isFullyExplored) {
  if (useIncrementalMergingSearch) {
    if (isFullyExplored) {
      /* TODO: add docs */
      executor->incrementalMergingSearcher->addState(es);
    } else {
      executor->incrementalMergingSearcher->internalSearcher->addState(es);
    }
  } else {
    executor->mergingSearcher->inCloseMerge.erase(es);
    executor->mergingSearcher->continueState(*es);
  }
  executor->terminateStateEarly(*es, reason);
  executor->interpreterHandler->decUnmergedExploredPaths();
}

void LoopHandler::discardState(ExecutionState *es,
                               const char *reason,
                               bool isFullyExplored) {
  if (find(openStates.begin(), openStates.end(), es) == openStates.end()) {
    /* the state reached the loop exit (suspended) */
    discardClosedState(es, reason, isFullyExplored);
  } else {
    discardOpenState(es, reason);
  }
}

void LoopHandler::addClosedState(ExecutionState *es,
                                 Instruction *mp) {
  ++closedStateCount;
  removeOpenState(es);
  pauseOpenState(es);

  auto i = mergeGroupsByExit.find(mp);
  if (i == mergeGroupsByExit.end()) {
    mergeGroupsByExit[mp].push_back(es);
  } else {
    StateSet &states = i->second;
    states.push_back(es);
  }

  /* otherwise, a state sneaked out somehow */
  assert(activeStates > 0);
  activeStates--;
  if (activeStates == 0) {
    releaseStates();
  }
}

static set<string> forceCFG = {
    "memmove",
    "memcpy",
    "memset",
    "strcpy",
    "strncpy",
};

bool LoopHandler::shouldForceCFGBasedMerging() {
  if (!RestrictPatternBaseMerging) {
    return false;
  }

  Function *f = loop->getHeader()->getParent();
  return forceCFG.find(f->getName()) != forceCFG.end();
}

bool LoopHandler::shouldUsePatternBasedMerging(vector<PatternMatch> &matches,
                                               vector<StateSet> &matchedStates) {
  for (unsigned i = 0; i < matches.size(); i++) {
    PatternMatch &pm = matches[i];
    StateSet &states = matchedStates[i];
    ExecutionState *merged = states[0];
    if (ExecutionState::shouldUsePatternBasedMerging(merged, states, pm, this)) {
      return true;
    }
  }

  return false;
}

void LoopHandler::splitStates(vector<MergeGroupInfo> &result) {
  if (SplitByPattern && !shouldForceCFGBasedMerging()) {
    for (const auto &i: mergeGroupsByExit) {
      const StateSet &states = i.second;

      set<uint32_t> ids;
      /* TODO: add this mapping to LoopHandler */
      map<uint32_t, ExecutionState *> m;
      for (ExecutionState *es : states) {
          ids.insert(es->getID());
          m[es->getID()] = es;
      }

      /* TODO: check forward as well */
      vector<PatternMatch> matches;
      if (UseForwardExtract) {
        extractPatterns(tree, ids, matches);
      } else {
        extractPatternsBackward(tree, ids, matches);
      }

      /* must be non-empty */
      assert(!matches.empty());

      vector<StateSet> matchedStates(matches.size());
      for (unsigned i = 0; i < matches.size(); i++) {
        PatternMatch &pm = matches[i];
        for (StateMatch &sm : pm.matches) {
          auto j = m.find(sm.stateID);
          assert(j != m.end());
          matchedStates[i].push_back(j->second);
        }
      }

      /* TODO: enable not only when there is one exit? */
      if (mergeGroupsByExit.size() == 1 && !shouldUsePatternBasedMerging(matches, matchedStates)) {
        klee_warning("pattern-based merging might not be beneficial");
        MergeGroupInfo groupInfo({MergeSubGroupInfo(states)});
        result.push_back(groupInfo);
        return;
      }

      if (matches.size() > MaxPatterns) {
        klee_warning("max patterns exceeded: %lu", matches.size());
        MergeGroupInfo groupInfo({MergeSubGroupInfo(states)});
        result.push_back(groupInfo);
      } else {
        if (SplitByCFG) {
          std::vector<MergeSubGroupInfo> subGroups;
          for (unsigned i = 0; i < matches.size(); i++) {
            PatternMatch &pm = matches[i];
            StateSet &states = matchedStates[i];
            subGroups.push_back(MergeSubGroupInfo(states, {pm}));
          }
          MergeGroupInfo groupInfo(subGroups);
          result.push_back(groupInfo);
        } else {
          for (unsigned i = 0; i < matches.size(); i++) {
            PatternMatch &pm = matches[i];
            StateSet &states = matchedStates[i];
            MergeGroupInfo groupInfo({MergeSubGroupInfo(states, {pm})});
            result.push_back(groupInfo);
          }
        }
      }
    }
  } else {
    for (const auto &i: mergeGroupsByExit) {
      const StateSet &states = i.second;
      MergeGroupInfo groupInfo({MergeSubGroupInfo(states)});
      result.push_back(groupInfo);
    }
  }
}

ExecutionState *LoopHandler::mergeSubGroup(MergeSubGroupInfo &info,
                                           bool isComplete) {
  StateSet &states = info.states;
  vector<PatternMatch> &matches = info.matches;

  if (MaxStatesToMerge != 0 && states.size() > MaxStatesToMerge) {
    return nullptr;
  }

  if (!UseOptimizedMerge) {
    return ExecutionState::mergeStates(states);
  }

  bool usePattern = OptimizeUsingQuantifiers && !matches.empty();
  return ExecutionState::mergeStatesOptimized(states,
                                              isComplete,
                                              usePattern,
                                              matches,
                                              this);
}

ExecutionState *LoopHandler::mergeGroup(MergeGroupInfo &groupInfo,
                                        bool isComplete) {
  StateSet states;
  for (MergeSubGroupInfo &subGroupInfo : groupInfo.subGroups) {
    for (ExecutionState *state : subGroupInfo.states) {
      states.push_back(state);
    }
  }

  vector<ExecutionState *> snapshots;
  if (ValidateMerge) {
    /* take snapshots before merging */
    for (ExecutionState *es : states) {
      snapshots.push_back(es->branch(true));
    }
  }

  /* TODO: pc or prevPC? */
  klee_message("merging %lu sub groups at %s:%u",
               groupInfo.subGroups.size(),
               states[0]->pc->info->file.data(),
               states[0]->pc->info->line);

  vector<ExecutionState *> subStates;
  for (MergeSubGroupInfo &subGroupInfo : groupInfo.subGroups) {
    ExecutionState *subState = mergeSubGroup(
      subGroupInfo,
      isComplete && groupInfo.subGroups.size() == 1
    );
    subStates.push_back(subState);
  }
  ExecutionState *merged = ExecutionState::mergeStates(subStates);
  if (!merged) {
    /* TODO: merged state might have merge side effects */
    char msg[1000];
    snprintf(msg,
             sizeof(msg),
             "unsupported merge: %s:%u",
             states[0]->prevPC->info->file.data(),
             states[0]->prevPC->info->line);
    klee_warning("%s", msg);

    for (ExecutionState *es : states) {
      es->suffixConstraints.clear();
      resumeClosedState(es);
    }

    return nullptr;
  }

  if (ValidateMerge) {
    assert(validateMerge(snapshots, merged));
    for (ExecutionState *es : snapshots) {
      delete es;
    }
  }

  /* TODO: increment before the merge? */
  merged->incMergeID();
  merged->hasPendingSnapshot = false;
  resumeClosedState(merged);

  /* TODO: why? */
  for (ExecutionState *es : states) {
    es->suffixConstraints.clear();
  }

  for (unsigned i = 1; i < states.size(); i++) {
    ExecutionState *es = states[i];
    discardClosedState(es, "Merge", true);
  }

  return merged;
}

void LoopHandler::releaseStates() {
  TimerStatIncrementer timer(stats::mergeTime);
  vector<MergeGroupInfo> groups;
  splitStates(groups);

  size_t total = 0;
  for (MergeGroupInfo &groupInfo: groups) {
    total += groupInfo.getStatesCount();
  }
  klee_message("splitting %lu states to %lu merging groups",
               total,
               groups.size());

  if (UseMergeTransformation || UseJoinTransformation) {
    dumpStats();
  }

  bool isComplete = (groups.size() == 1) && (earlyTerminated == 0);
  unsigned groupID = 0;
  for (MergeGroupInfo &groupInfo: groups) {
    ExecutionState *merged = mergeGroup(groupInfo, isComplete);
    executor->collectMergeStats(*merged);
    if (groups.size() == 1) {
      klee_message("merged %lu states (complete = %u)",
                   groupInfo.getStatesCount(),
                   isComplete);
    } else {
      klee_message("merged %lu states (complete = %u, group = %u)",
                   groupInfo.getStatesCount(),
                   isComplete,
                   groupID);
    }
    groupID++;
  }

  mergeGroupsByExit.clear();

  /* the snapshots hold a reference to the loop hander... */
  tree.clear();

  if (useIncrementalMergingSearch) {
    assert(executor->incrementalMergingSearcher->internalSearcher->empty());
  }
}

/* TODO: update openStates? */
void LoopHandler::markEarlyTerminated(ExecutionState &state) {
  earlyTerminated++;
  assert(activeStates > 0);
  activeStates--;
  if (activeStates == 0) {
    releaseStates();
  }
}

unsigned LoopHandler::getEarlyTerminated() {
  return earlyTerminated;
}

bool LoopHandler::canMergeNodes(ExecTreeNode *n1, ExecTreeNode *n2) {
  ExecTreeNode *ancestor = tree.getNearestAncestor(n1, n2);
  return tree.isSinglePath(n1, ancestor) || tree.isSinglePath(n2, ancestor);
}

bool LoopHandler::shouldMerge(ExecutionState &s1, ExecutionState &s2) {
  vector<ExecutionState *> states = {&s1, &s2};
  set<const MemoryObject*> mutated;
  if (!ExecutionState::canMerge(states, mutated)) {
    return false;
  }

  if (!ExecutionState::compareStack(s1, s2)) {
    return false;
  }

  if (!ExecutionState::compareHeap(s1, s2, mutated)) {
    return false;
  }

  return true;
}

bool LoopHandler::discardStateByID(unsigned id) {
  /* search in the open states */
  for (ExecutionState *es : openStates) {
    if (es->getID() == id) {
      /* TODO: remove from openStates? */
      discardState(es, "Incremental Merge", false);
      return true;
    }
  }

  /* search in the closed states */
  for (auto &i : mergeGroupsByExit) {
    StateSet &states = i.second;
    auto j = states.begin();
    while (j != states.end()) {
      ExecutionState *es = *j;
      if (es->getID() == id) {
        discardState(es, "Incremental Merge", false);
        states.erase(j);
        return true;
      }
      j++;
    }
  }

  return false;
}

void LoopHandler::discardSubTree(ExecTreeNode *src,
                                 ExecTreeNode *ancestor) {
  set<unsigned> ids;
  vector<ExecTreeNode *> nodes;

  tree.getReachable(src, nodes);
  for (ExecTreeNode *n : nodes) {
    if (n->isLeaf()) {
      /* an intermediate state is not associated with an active state */
      ids.insert(n->stateID);
    }
  }

  /* remove the active states */
  for (unsigned id : ids) {
    bool found = discardStateByID(id);
    assert(found);
  }

  tree.removeSubTree(src, ancestor);
}

void LoopHandler::setSuffixConstraints(ExecutionState *merged,
                                       ExecTreeNode *ancestor,
                                       ref<Expr> condition) {
  list<ref<Expr>> prefixConditions;
  ExecTreeNode *n = ancestor;
  while (n) {
    prefixConditions.push_front(n->e);
    n = n->parent;
  }

  merged->clearSuffixConstraints();
  for (ref<Expr> e : prefixConditions) {
    merged->addSuffixConstraint(e);
  }
  merged->addSuffixConstraint(condition);
}

void LoopHandler::mergeNodes(ExecTreeNode *n1,
                             ExecTreeNode *n2,
                             ExecutionState *s1,
                             ExecutionState *s2) {
  if (DebugMergeTransformation) {
    tree.dumpGMLToFile("before-merge");
  }

  vector<ExecutionState *> states = {s1, s2};
  /* TODO: use ExecutionState::mergeStatesOptimized? */
  ExecutionState *merged = ExecutionState::mergeStates(states);

  /* clone the merged state (to avoid using the snapshots) */
  merged = merged->branch();
  /* TODO: add docs */
  merged->hasPendingSnapshot = true;

  executor->processTree->replaceNode(n1->ptreeNode, merged);

  /* find nearest ancestor */
  ExecTreeNode *ancestor = tree.getNearestAncestor(n1, n2);
  ref<Expr> pc1 = tree.getPC(ancestor, n1);
  ref<Expr> pc2 = tree.getPC(ancestor, n2);
  ref<Expr> condition = OrExpr::create(pc1, pc2);

  setSuffixConstraints(merged, ancestor, condition);

  /* remove paths to nodes */
  discardSubTree(n1, ancestor);
  discardSubTree(n2, ancestor);

  /* TODO: update pc for the snapshot? */
  ExecutionState *mergedSnapshot = merged->branch(true);
  if (!ancestor->left) {
    tree.setLeft(ancestor, *merged, condition, mergedSnapshot);
  } else if (!ancestor->right) {
    tree.setRight(ancestor, *merged, condition, mergedSnapshot);
  } else {
    assert(0);
  }

  /* add new path */
  executor->addedStates.push_back(merged);

  if (DebugMergeTransformation) {
    tree.dumpGMLToFile("after-merge");
  }
}

bool LoopHandler::mergeNodes(ExecTreeNode *n1, ExecTreeNode *n2) {
  if (!canMergeNodes(n1, n2)) {
    return false;
  }

  /* TODO: check only the last snapshot of s1? */
  for (ExecutionState *s1 : n1->snapshots) {
    for (ExecutionState *s2 : n2->snapshots) {
      if (shouldMerge(*s1, *s2)) {
        mergeNodes(n1, n2, s1, s2);
        return true;
      }
    }
  }

  return false;
}

bool LoopHandler::mergeIntermediateState(ExecTreeNode *target) {
  list<ExecTreeNode *> worklist;

  ExecTreeNode *n = target;
  while (n->parent) {
    ExecTreeNode *sibling = n->getSibling();
    if (sibling) {
      /* prioritize earlier states */
      worklist.push_front(sibling);
    }
    n = n->parent;
  }

  for (ExecTreeNode *start : worklist) {
    /* prioritize earlier states */
    ExecTreeBFSIterator iter(tree, start);
    while (iter.hasNext()) {
      ExecTreeNode *n = iter.next();
      if (mergeNodes(target, n)) {
        return true;
      }
    }
  }

  return false;
}

bool LoopHandler::runMergeTransformationNaive() {
  if (!UseMergeTransformation) {
    return false;
  }

  bool changed = false;
  bool retry;

  do {
    retry = false;
    ExecTreeBFSIterator iter(tree);
    while (iter.hasNext()) {
      ExecTreeNode *n = iter.next();
      if (n != tree.root) {
        if (mergeIntermediateState(n)) {
          mergeCount++;
          retry = true;
          changed = true;
          break;
        }
      }
    }
  } while (retry);

  return changed;
}

bool LoopHandler::runMergeTransformation() {
  if (!UseMergeTransformation) {
    return false;
  }

  bool changed = false;

  for (ExecTreeNode *n : tree.nodesToMerge) {
    if (mergeIntermediateState(n)) {
      mergeCount++;
      changed = true;
    }
  }
  tree.nodesToMerge.clear();

  return changed;
}

bool LoopHandler::runJoinTransformation() {
  if (!UseJoinTransformation) {
    return false;
  }

  bool changed = false;
  bool retry;

  do {
    retry = false;
    for (ExecTreeNode *n : tree.nodes) {
      if (tree.join(n)) {
        joinCount++;
        retry = true;
        break;
      }
    }
  } while (retry);

  return changed;
}

bool LoopHandler::transform() {
  TimerStatIncrementer timer(stats::mergeTime);
  bool changed = false;

  if (runMergeTransformation()) {
    changed = true;
    runJoinTransformation();
  }

  shouldTransform = false;
  return changed;
}

bool LoopHandler::validateMerge(vector<ExecutionState *> &snapshots,
                                ExecutionState *merged) {
  ExecutionState *expected = ExecutionState::mergeStates(snapshots);
  return ExecutionState::areEquiv(executor->solver,
                                  merged,
                                  expected,
                                  !OptimizeUsingQuantifiers);
}

void LoopHandler::dumpStats() const {
  klee_message("merge transformations: %lu", mergeCount);
  klee_message("join transformations: %lu", joinCount);
}

}
