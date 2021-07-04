#ifndef KLEE_EXEC_TREE_REWRITE_H
#define KLEE_EXEC_TREE_REWRITE_H

#include "ExecTree.h"

#include <klee/Expr/Expr.h>

#include <vector>

namespace klee {

void findRepeatingSubtrees(ExecTree &t,
                           std::vector<ExecTreeNode *> &result);

}

#endif
