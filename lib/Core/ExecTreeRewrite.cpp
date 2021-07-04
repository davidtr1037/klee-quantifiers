#include "ExecTree.h"
#include "ExecTreeRewrite.h"

#include <vector>
#include <list>

using namespace klee;
using namespace llvm;

void klee::findRepeatingSubtrees(ExecTree &t,
                                 std::vector<ExecTreeNode *> &result) {
  std::set<unsigned> allHashes;
  std::set<unsigned> repeatingHashes;
  std::list<ExecTreeNode *> worklist;

  /* collect the hash values */
  worklist.push_back(t.root);
  while (!worklist.empty()) {
    ExecTreeNode *n = worklist.front();
    worklist.pop_front();
    if (!n->isLeaf()) {
      unsigned hash = n->getSubTreeHash();
      if (allHashes.find(hash) != allHashes.end()) {
        repeatingHashes.insert(hash);
      }
      allHashes.insert(n->getSubTreeHash());
    }

    if (!n->isLeaf()) {
      worklist.push_back(n->left);
      worklist.push_back(n->right);
    }
  }

  /* mark the repeating subtrees */
  unsigned hash = 0;

  worklist.push_back(t.root);
  while (!worklist.empty()) {
    ExecTreeNode *n = worklist.front();
    worklist.pop_front();
    if (!n->isLeaf()) {
      if (repeatingHashes.find(n->getSubTreeHash()) != repeatingHashes.end()) {
        if (result.empty()) {
          hash = n->getSubTreeHash();
          result.push_back(n);
        } else {
          if (n->getSubTreeHash() == hash) {
            result.push_back(n);
          }
        }
      }
    }

    if (!n->isLeaf()) {
      worklist.push_back(n->left);
      worklist.push_back(n->right);
    }
  }
}
