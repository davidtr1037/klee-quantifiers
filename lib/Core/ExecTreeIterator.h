#ifndef KLEE_EXEC_TREE_ITERATOR_H
#define KLEE_EXEC_TREE_ITERATOR_H

#include "ExecTree.h"
#include "Pattern.h"

#include <list>

namespace klee {

class ExecTreeIterator {
public:

  ExecTreeIterator(ExecTree &t) : t(t) {
    current = nullptr;
  }

  /* TODO: just pass the hash value? */
  void next(const Symbol &s);

  bool hasNext() const;

  /* TODO: inline? */
  ExecTreeNode *getCurrent() const;

private:

  ExecTree &t;
  ExecTreeNode *current;
};

class ExecTreeBFSIterator {
public:

  ExecTreeBFSIterator(ExecTree &t, ExecTreeNode *start = nullptr) : t(t) {
    ExecTreeNode *initial = start ? start : t.root;
    worklist.push_back(initial);
  }

  ExecTreeNode *next();

  bool hasNext() const;

private:

  ExecTree &t;
  std::list<ExecTreeNode *> worklist;
};

}

#endif
