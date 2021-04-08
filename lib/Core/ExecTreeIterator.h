#ifndef KLEE_EXEC_TREE_ITERATOR_H
#define KLEE_EXEC_TREE_ITERATOR_H

#include "ExecTree.h"
#include "Pattern.h"

namespace klee {

class ExecTreeIterator {
public:

  ExecTreeIterator(ExecTree &t) : t(t) {
    current = nullptr;
  }

  /* TODO: just pass the hash value? */
  void next(Symbol &s);

  bool hasNext() const;

  /* TODO: inline? */
  ExecTreeNode *getCurrent() const;

private:

  ExecTree &t;
  ExecTreeNode *current;
};

}

#endif
