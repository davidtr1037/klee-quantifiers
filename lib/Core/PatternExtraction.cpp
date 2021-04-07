#include "PatternExtraction.h"

void klee::extractPatterns(ExecTree &t, std::vector<PatternMatch> &matches) {
  std::vector<std::pair<ExecTreeNode *, PatternInstance>> worklist;
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
      /* TODO: */
      pi.dump();
    }
  }
}
