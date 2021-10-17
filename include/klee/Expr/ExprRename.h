#ifndef KLEE_EXPRRENAME_H
#define KLEE_EXPRRENAME_H

#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"

#include <vector>
#include <map>

namespace klee {

struct ArrayCompare {
  bool operator()(const Array *a, const Array *b) const {
    return a->id < b->id;
  }
};

typedef std::map<const Array *, const Array *, ArrayCompare> ArrayMap;

const Array *cloneArray(const Array *from, unsigned i);

ref<Expr> rename(const ref<Expr> e, const ArrayMap &map);

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
