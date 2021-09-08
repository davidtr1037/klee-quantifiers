#include "ExecTreeIterator.h"

using namespace klee;
using namespace llvm;

void ExecTreeIterator::next(const Symbol &s) {
  if (current == nullptr) {
    assert(t.root->getHash() == s.hash);
    current = t.root;
  } else {
    /* TODO: remove later? */
    assert(hasNext());

    if (current->left && current->left->getHash() == s.hash) {
      current = current->left;
    } else if (current->right && current->right->getHash() == s.hash) {
      current = current->right;
    } else {
      assert(0);
    }
  }
}

bool ExecTreeIterator::hasNext() const {
  if (current == nullptr) {
    return t.root != nullptr;
  } else {
    /* TODO: is it enough? */
    return !current->isLeaf();
  }
}

ExecTreeNode *ExecTreeIterator::getCurrent() const {
  return current;
}

ExecTreeNode *ExecTreeBFSIterator::next() {
  if (!hasNext()) {
    return nullptr;
  }

  ExecTreeNode *n = worklist.front();
  worklist.pop_front();

  if (n->left) {
    worklist.push_back(n->left);
  }
  if (n->right) {
    worklist.push_back(n->right);
  }

  return n;
}

bool ExecTreeBFSIterator::hasNext() const {
  return !worklist.empty();
}
