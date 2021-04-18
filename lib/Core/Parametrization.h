#ifndef KLEE_PARAMETRIZATION_H
#define KLEE_PARAMETRIZATION_H

#include "PatternExtraction.h"
#include "ExecTree.h"
#include "TimingSolver.h"

#include <klee/Expr/Expr.h>
#include <klee/Expr/ArrayCache.h>

#include <vector>

namespace klee {

struct SMTEquation {

  SMTEquation(ref<Expr> e, unsigned k) : e(e), k(k) {

  }

  ref<Expr> e;
  unsigned k;
};

typedef std::vector<SMTEquation> SMTEquationSystem;

/* TODO: rename? */
struct ParametrizedExpr {
  ParametrizedExpr() : e(nullptr), parameter(nullptr) {

  }

  ParametrizedExpr(ref<Expr> e, ref<Expr> parameter) :
    e(e), parameter(parameter) {

  }

  ref<Expr> e;
  ref<Expr> parameter;
};

const Array *getArray(const std::string &name, uint64_t size, bool modelAsBV = false);

ref<Expr> getSymbolicValue(const Array *array, unsigned size);

ref<Expr> extractPrefixConstraint(ExecTree &t,
                                  PatternMatch &pm);

void extractEquationsForCore(ExecTree &t,
                             PatternMatch &pm,
                             std::vector<SMTEquationSystem> &result);

void extractEquationsForSuffix(ExecTree &t,
                               PatternMatch &pm,
                               std::vector<SMTEquationSystem> &result);

bool solveEquationSystem(SMTEquationSystem &system,
                         TimingSolver &solver,
                         ParametrizedExpr &result);

}

#endif
