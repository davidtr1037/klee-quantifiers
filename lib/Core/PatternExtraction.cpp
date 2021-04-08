#include "PatternExtraction.h"


using namespace llvm;

void klee::extractPatterns(ExecTree &t, std::vector<PatternMatch> &result) {
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
      bool found = false;
      for (PatternMatch &pm : matches) {
        unsigned count;
        if (pi.isInstanceOf(pm.pattern, count)) {
          pm.matches.push_back(StateMatch(count));
          break;
        }
      }

      if (!found) {
        PatternMatch pm;
        pm.pattern = Pattern(pi.prefix, pi.core, pi.suffix);
        pm.matches.push_back(StateMatch(pi.count));
        matches.push_back(pm);
      }
    }
  }

  /* try to match the non-repetitive patterns */
  for (PatternMatch &pm : matches) {
    if (pm.pattern.hasCore()) {
      result.push_back(pm);
    } else {
      /* no repetitions, only prefix */
      PatternInstance pi(pm.pattern.prefix);

      bool found = false;
      for (PatternMatch &other : matches) {
        unsigned count;
        if (pi.isInstanceOf(other.pattern, count)) {
          other.matches.push_back(StateMatch(count));
          found = true;
          break;
        }
      }

      if (!found) {
        result.push_back(pm);
      }
    }
  }

  for (PatternMatch &pm : result) {
    errs() << "pattern:\n";
    pm.pattern.dump();
    for (auto &sm : pm.matches) {
      errs() << "-- count " << sm.count << "\n";
    }
  }
}
