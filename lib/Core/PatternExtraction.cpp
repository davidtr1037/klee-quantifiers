#include "PatternExtraction.h"

#include <algorithm>

using namespace llvm;
using namespace klee;

static bool compare(StateMatch &sm1, StateMatch &sm2) {
  return sm1.count < sm2.count;
}

PatternMatch::PatternMatch(const PatternInstance &pi) {
  pattern = Pattern(pi.prefix, pi.core, pi.suffix);
}

void PatternMatch::addStateMatch(const StateMatch &sm) {
  if (!pattern.hasCore()) {
    /* if has no core, only one match is possible */
    assert(matches.empty());
  }

  matches.push_back(sm);
}

void PatternMatch::dump() const {
  errs() << "pattern:\n";
  pattern.dump();
  for (auto &sm : matches) {
    errs() << "-- count " << sm.count << "\n";
  }
}

static bool addPatttern(std::vector<PatternMatch> &matches,
                        PatternInstance &pi,
                        uint32_t stateID) {
  for (PatternMatch &pm : matches) {
    unsigned count;
    if (pi.isInstanceOf(pm.pattern, count)) {
      pm.matches.push_back(StateMatch(stateID, count));
      return true;
    }
  }

  PatternMatch pm(pi);
  pm.addStateMatch(StateMatch(stateID, pi.count));
  matches.push_back(pm);
  return false;
}

static void handleLeaf(ExecTreeNode *n,
                       PatternInstance &pi,
                       std::vector<PatternMatch> &matches) {
  addPatttern(matches, pi, n->stateID);
}

static void unifyMatches(std::vector<PatternMatch> &matches,
                         std::vector<PatternMatch> &result) {
  for (PatternMatch &pm : matches) {
    if (pm.pattern.hasCore()) {
      result.push_back(pm);
    }
  }

  for (PatternMatch &pm : matches) {
    if (!pm.pattern.hasCore()) {
      /* no repetitions, only prefix */
      PatternInstance pi(pm.pattern.prefix);

      /* get state id */
      assert(pm.matches.size() == 1);
      addPatttern(result, pi, pm.matches[0].stateID);
    }
  }
}

void klee::extractPatterns(ExecTree &t,
                           std::set<uint32_t> &ids,
                           std::vector<PatternMatch> &result) {
  std::vector<PatternMatch> matches;
  std::vector<std::pair<ExecTreeNode *, PatternInstance>> worklist;

  /* initialize the worklist */
  worklist.push_back(std::make_pair(t.root, PatternInstance()));

  while (!worklist.empty()) {
    auto p = worklist.back();
    worklist.pop_back();

    ExecTreeNode *n = p.first;
    PatternInstance pi = p.second;
    Symbol s(n->e->shapeHash());
    pi.addSymbol(s);

    if (!n->isLeaf()) {
      worklist.push_back(std::make_pair(n->left, pi));
      worklist.push_back(std::make_pair(n->right, pi));
    } else {
      if (ids.find(n->stateID) != ids.end()) {
        handleLeaf(n, pi, matches);
      }
    }
  }

  /* try to match the non-repetitive patterns */
  unifyMatches(matches, result);
  for (PatternMatch &pm : result) {
    /* we sort, to help in finding distinct terms */
    /* TODO: something more efficient? */
    std::sort(pm.matches.begin(), pm.matches.end(), compare);
  }

  for (PatternMatch &pm : result) {
    errs() << "pattern:\n";
    pm.pattern.dump();
    for (auto &sm : pm.matches) {
      errs() << "-- count " << sm.count << "\n";
    }
  }
}
