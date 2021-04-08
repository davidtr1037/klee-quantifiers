#include "ExecTreeIterator.h"

using namespace klee;
using namespace llvm;

void ExecTreeIterator::next(Symbol &s) {
  if (current == nullptr) {
    assert(t.root->e->shapeHash() == s.hash);
    current = t.root;
  } else {
    /* TODO: remove later? */
    assert(hasNext());

    if (current->left->e->shapeHash() == s.hash) {
      current = current->left;
    } else if (current->right->e->shapeHash() == s.hash) {
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
