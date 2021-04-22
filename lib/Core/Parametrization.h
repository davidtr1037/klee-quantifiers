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
  ParametrizedExpr() : e(nullptr), parameter(nullptr), array(nullptr) {

  }

  ParametrizedExpr(ref<Expr> e, ref<Expr> parameter, const Array *array) :
    e(e), parameter(parameter), array(array) {

  }

  ref<Expr> e;
  ref<Expr> parameter;
  const Array *array;
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

void extractEquationsForValue(ExecTree &t,
                              PatternMatch &pm,
                              State2Value &valuesMap,
                              SMTEquationSystem &system);

bool solveEquationSystem(SMTEquationSystem &system,
                         TimingSolver &solver,
                         uint32_t id,
                         ParametrizedExpr &result);

}

#endif
