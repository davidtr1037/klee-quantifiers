#ifndef KLEE_EXPRRENAME_H
#define KLEE_EXPRRENAME_H

#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"

#include <vector>

namespace klee {

typedef std::map<const Array *, const Array *> ArrayMap;

void rename(const Query &query,
            ConstraintSet &constraints,
            ref<Expr> &expr,
            ArrayMap &map);

void rename(const Query &query,
            ConstraintSet &constraints,
            ref<Expr> &expr);

void rename(const Query &query,
            const std::vector<const Array *> &objects,
            ConstraintSet &constraints,
            ref<Expr> &expr,
            std::vector<const Array *> &renamedObjects);

}

#endif
