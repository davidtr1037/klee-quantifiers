#ifndef KLEE_EXEC_TREE_ITERATOR_H
#define KLEE_EXEC_TREE_ITERATOR_H

#include "ExecTree.h"
#include "Pattern.h"

namespace klee {

class ExecTreeIterator {
public:

  ExecTreeIterator(ExecTree &t) {
    current = t.root;
  }

  /* TODO: just pass the hash value? */
  void next(Symbol &s);

  bool hasNext() const;

  inline ExecTreeNode *getCurrent() const;

private:

  ExecTreeNode *current;
};

}

#endif
