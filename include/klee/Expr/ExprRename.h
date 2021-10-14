#ifndef KLEE_EXPRRENAME_H
#define KLEE_EXPRRENAME_H

#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"

#include <vector>

namespace klee {

//class Array;
//class Expr;
//class ReadExpr;
//template<typename T> class ref;

typedef std::map<const Array *, const Array *> ArrayMap;

ArrayMap rename(const Query &query,
                ConstraintSet &constraints,
                ref<Expr> &expr);

void rename(const Query &query,
            const std::vector<const Array *> &objects,
            ConstraintSet &constraints,
            ref<Expr> &expr,
            std::vector<const Array *> &renamedObjects);

}

#endif
