#ifndef KLEE_PARAMETRIZATION_H
#define KLEE_PARAMETRIZATION_H

#include "PatternExtraction.h"
#include "ExecTree.h"

#include <klee/Expr/Expr.h>

#include <vector>

namespace klee {

struct SMTEquation {

  SMTEquation(ref<Expr> e, unsigned k) : e(e), k(k) {

  }

  ref<Expr> e;
  unsigned k;
};

typedef std::vector<SMTEquation> SMTEquationSystem;

void extractEquationsForCore(ExecTree &t,
                             PatternMatch &pm,
                             std::vector<SMTEquationSystem> &result);

void extractEquationsForSuffix(ExecTree &t,
                               PatternMatch &pm,
                               std::vector<SMTEquationSystem> &result);

}

#endif
