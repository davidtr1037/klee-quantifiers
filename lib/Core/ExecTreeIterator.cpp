#include "ExecTreeIterator.h"

using namespace klee;

void ExecTreeIterator::next(Symbol &s) {
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

bool ExecTreeIterator::hasNext() const {
  /* TODO: is it enough? */
  return !current->isLeaf();
}

inline ExecTreeNode *ExecTreeIterator::getCurrent() const {
  return current;
}
