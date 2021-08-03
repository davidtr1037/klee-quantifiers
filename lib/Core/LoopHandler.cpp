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
    "use-loop-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> DebugLoopHandler(
    "debug-loop-handler", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseOptimizedMerge(
    "use-optimized-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> ValidateMerge(
    "validate-merge", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<unsigned> MaxStatesToMerge(
    "max-states-to-merge", cl::init(10000),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> SplitByPattern(
    "split-by-pattern", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseForwardExtract(
    "use-forward-extract", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseMergeTransformation(
    "use-merge-transformation", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

cl::opt<bool> UseJoinTransformation(
    "use-join-transformation", cl::init(false),
    cl::desc(""),
    cl::cat(klee::LoopCat));

LoopHandler::LoopHandler(Executor *executor, ExecutionState *es, Loop *loop)
    : closedStateCount(0),
      activeStates(0),
      earlyTerminated(0),
      executor(executor),
      solver(executor->solver),
      loop(loop),
      tree(es->getID()),
      canUseExecTree(true),
      shouldTransform(false),
      mergeCount(0),
      joinCount(0) {
  assert(loop);
  addOpenState(es);
  for (ref<Expr> e : es->constraints) {
    initialConstraints.push_back(e);
  }
}

LoopHandler::~LoopHandler() {
  if (executor->haltExecution) {
    /* if execution is interrupted, may contain unreleased states */
    return;
  }

  assert(activeStates == 0);
  for (auto &i: mergeGroupsByExit) {
    vector<ExecutionState *> &states = i.second;
    assert(states.empty());
  }
}

void LoopHandler::addOpenState(ExecutionState *es){
  openStates.push_back(es);
  activeStates++;
}

void LoopHandler::removeOpenState(ExecutionState *es) {
  auto it = std::find(openStates.begin(), openStates.end(), es);
  assert(it != openStates.end());
  std::swap(*it, openStates.back());
  openStates.pop_back();
}

void LoopHandler::addClosedState(ExecutionState *es,
                                 Instruction *mp) {
  ++closedStateCount;
  removeOpenState(es);

  auto i = mergeGroupsByExit.find(mp);
  if (i == mergeGroupsByExit.end()) {
    mergeGroupsByExit[mp].push_back(es);
  } else {
    MergeGroup &states = i->second;
    states.push_back(es);
  }

  executor->mergingSearcher->pauseState(*es);

  /* otherwise, a state sneaked out somehow */
  assert(activeStates > 0);
  activeStates--;
  if (activeStates == 0) {
    releaseStates();
  }
}

void LoopHandler::splitStates(std::vector<MergeGroup> &result) {
  if (SplitByPattern) {
    for (auto &i: mergeGroupsByExit) {
      MergeGroup &states = i.second;

      std::set<uint32_t> ids;
      /* TODO: add this mapping to LoopHandler */
      std::map<uint32_t, ExecutionState *> m;
      for (ExecutionState *es : states) {
          ids.insert(es->getID());
          m[es->getID()] = es;
      }

      /* TODO: check forward as well */
      std::vector<PatternMatch> matches;
      if (UseForwardExtract) {
        extractPatterns(tree, ids, matches);
      } else {
        extractPatternsBackward(tree, ids, matches);
      }

      for (PatternMatch &pm : matches) {
        MergeGroup states;
        for (StateMatch &sm : pm.matches) {
          auto i = m.find(sm.stateID);
          assert(i != m.end());
          states.push_back(i->second);
        }
        result.push_back(states);
      }
    }
  } else {
    for (auto &i: mergeGroupsByExit) {
      MergeGroup &states = i.second;
      result.push_back(states);
    }
  }
}

void LoopHandler::releaseStates() {
  std::vector<MergeGroup> groups;
  splitStates(groups);
  klee_message("splitting to %lu merging groups", groups.size());
  dumpStats();

  unsigned groupId = 0;
  for (MergeGroup &states: groups) {
    vector<ExecutionState *> snapshots;
    if (ValidateMerge) {
      /* take snapshots before merging */
      for (ExecutionState *es : states) {
        snapshots.push_back(es->branch(true));
      }
    }

    ExecutionState *merged = nullptr;
    bool isComplete = (groups.size() == 1) && (earlyTerminated == 0);
    /* TODO: pc or prevPC? */
    klee_message("merging at %s:%u",
                 states[0]->pc->info->file.data(),
                 states[0]->pc->info->line);

    if (MaxStatesToMerge == 0 || states.size() < MaxStatesToMerge) {
      if (UseOptimizedMerge) {
        std::vector<PatternMatch> matches;
        if (OptimizeUsingQuantifiers) {
          std::set<uint32_t> ids;
          for (ExecutionState *es : states) {
            ids.insert(es->getID());
          }
          /* TODO: avoid calling twice */
          if (UseForwardExtract) {
            extractPatterns(tree, ids, matches);
          } else {
            extractPatternsBackward(tree, ids, matches);
          }
        }

        merged = ExecutionState::mergeStatesOptimized(states,
                                                      isComplete,
                                                      OptimizeUsingQuantifiers,
                                                      matches,
                                                      this);
      } else {
        merged = ExecutionState::mergeStates(states);
      }
    }
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
        executor->mergingSearcher->inCloseMerge.erase(es);
        es->suffixConstraints.clear();
        executor->mergingSearcher->continueState(*es);
      }

      /* TODO: refactor... */
      continue;
    } else {
      merged->hasPendingSnapshot = false;
    }

    if (ValidateMerge) {
      assert(validateMerge(snapshots, merged));
      for (ExecutionState *es : snapshots) {
        delete es;
      }
    }

    executor->mergingSearcher->continueState(*merged);
    executor->collectMergeStats(*merged);
    if (groups.size() == 1) {
      klee_message("merged %lu states (complete = %u)",
                   states.size(),
                   isComplete);
    } else {
      klee_message("merged %lu states (complete = %u, group = %u)",
                   states.size(),
                   isComplete,
                   groupId);
    }

    for (ExecutionState *es : states) {
      executor->mergingSearcher->inCloseMerge.erase(es);
      /* TODO: why? */
      es->suffixConstraints.clear();
    }

    for (unsigned i = 1; i < states.size(); i++) {
      ExecutionState *es = states[i];
      executor->mergingSearcher->continueState(*es);
      executor->terminateStateEarly(*es, "Merge");
      executor->interpreterHandler->decUnmergedExploredPaths();
    }

    groupId++;
  }
  mergeGroupsByExit.clear();
  /* the snapshots hold a reference to the loop hander... */
  tree.clear();
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

bool LoopHandler::shouldMerge(ExecutionState &s1, ExecutionState &s2) {
  std::vector<ExecutionState *> states = {&s1, &s2};
  std::set<const MemoryObject*> mutated;
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

void LoopHandler::discardState(ExecutionState *es) {
  if (std::find(openStates.begin(), openStates.end(), es) == openStates.end()) {
    /* the state reached the loop exit (suspended) */
    executor->mergingSearcher->inCloseMerge.erase(es);
    executor->mergingSearcher->continueState(*es);
  }
  executor->terminateStateEarly(*es, "IntermediateMerge");
  executor->interpreterHandler->decUnmergedExploredPaths();
}

bool LoopHandler::discardStateByID(unsigned id) {
  /* search in the open states */
  for (ExecutionState *es : openStates) {
    if (es->getID() == id) {
      /* TODO: remove from openStates? */
      discardState(es);
      return true;
    }
  }

  /* search in the closed states */
  for (auto &i : mergeGroupsByExit) {
    MergeGroup &group = i.second;
    auto j = group.begin();
    while (j != group.end()) {
      ExecutionState *es = *j;
      if (es->getID() == id) {
        discardState(es);
        group.erase(j);
        return true;
      }
      j++;
    }
  }

  return false;
}

void LoopHandler::discardSubTree(ExecTreeNode *src,
                                 ExecTreeNode *ancestor) {
  std::set<unsigned> ids;
  std::vector<ExecTreeNode *> nodes;

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

void LoopHandler::mergeNodes(ExecTreeNode *n1,
                             ExecTreeNode *n2,
                             ExecutionState *s1,
                             ExecutionState *s2) {
  /* TODO: use ExecutionState::mergeStatesOptimized */
  std::vector<ExecutionState *> states = {s1, s2};
  ExecutionState *merged = ExecutionState::mergeStates(states);
  /* clone the merged state */
  merged = merged->branch();

  executor->processTree->replaceNode(n1->ptreeNode, merged);

  /* find nearest ancestor */
  ExecTreeNode *ancestor = tree.getNearestAncestor(n1, n2);
  ref<Expr> pc1 = tree.getPC(ancestor, n1);
  ref<Expr> pc2 = tree.getPC(ancestor, n2);

  /* remove paths to nodes */
  discardSubTree(n1, ancestor);
  discardSubTree(n2, ancestor);

  /* TODO: update pc for the snapshot? */
  ExecutionState *mergedSnapshot = merged->branch(true);
  ref<Expr> condition = OrExpr::create(pc1, pc2);
  if (!ancestor->left) {
    tree.setLeft(ancestor, *merged, condition, mergedSnapshot);
  } else if (!ancestor->right) {
    tree.setRight(ancestor, *merged, condition, mergedSnapshot);
  } else {
    /* TODO: unsupported */
    assert(0);
  }

  /* add new path */
  executor->addedStates.push_back(merged);
}

bool LoopHandler::mergeNodes(ExecTreeNode *n1, ExecTreeNode *n2) {
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
  std::list<ExecTreeNode *> worklist;

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

/* TODO: use the merged node to optimize the search? */
bool LoopHandler::mergeIntermediateStates() {
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

bool LoopHandler::joinIntermediateStates() {
  if (!UseJoinTransformation) {
    return false;
  }

  bool changed = false;
  bool retry;

  do {
    retry = false;
    for (ExecTreeNode *n : tree.nodes) {
      if (n->parent && !n->parent->isComplete()) {
        if (tree.join(n)) {
          joinCount++;
        }
        retry = true;
        break;
      }
    }
  } while (retry);

  return changed;
}

bool LoopHandler::transform() {
  bool changed = false;

  if (mergeIntermediateStates()) {
    changed = true;
    joinIntermediateStates();
  }

  shouldTransform = false;
  return changed;
}

bool LoopHandler::validateMerge(std::vector<ExecutionState *> &snapshots,
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
