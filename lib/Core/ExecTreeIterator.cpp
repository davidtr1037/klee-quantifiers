#include "ExecTreeIterator.h"

using namespace klee;
using namespace llvm;

void ExecTreeIterator::next(const Symbol &s) {
  if (current == nullptr) {
    assert(t.getNodeHash(t.root) == s.hash);
    current = t.root;
  } else {
    /* TODO: remove later? */
    assert(hasNext());

    if (current->left && t.getNodeHash(current->left) == s.hash) {
      current = current->left;
    } else if (current->right && t.getNodeHash(current->right) == s.hash) {
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
