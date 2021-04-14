#include "PatternExtraction.h"


using namespace llvm;
using namespace klee;

static bool addPatttern(std::vector<PatternMatch> &matches,
                        PatternInstance &pi) {
  for (PatternMatch &pm : matches) {
    unsigned count;
    if (pi.isInstanceOf(pm.pattern, count)) {
      pm.matches.push_back(StateMatch(count));
      return true;
    }
  }

  PatternMatch pm(pi);
  matches.push_back(pm);
  return false;
}

static void handleLeaf(PatternInstance &pi, std::vector<PatternMatch> &matches) {
  addPatttern(matches, pi);
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
      addPatttern(result, pi);
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
        handleLeaf(pi, matches);
      }
    }
  }

  /* try to match the non-repetitive patterns */
  unifyMatches(matches, result);

  for (PatternMatch &pm : result) {
    errs() << "pattern:\n";
    pm.pattern.dump();
    for (auto &sm : pm.matches) {
      errs() << "-- count " << sm.count << "\n";
    }
  }
}
