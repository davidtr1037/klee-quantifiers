#ifndef KLEE_EMATCHING_H
#define KLEE_EMATCHING_H

#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"

namespace klee {

/* TODO: make internal */
struct EqAssertion {
  /* TODO: add bound variable? */
  ref<EqExpr> assertion;
  bool isNegated;

  EqAssertion(ref<EqExpr> assertion, bool isNegated) :
    assertion(assertion), isNegated(isNegated) {

  }

  void findImpliedNegatingTerms(std::vector<ref<Expr>> &terms);

  void findNegatingTermsForRead(ref<ReadExpr> body,
                                ref<ReadExpr> e,
                                std::vector<ref<Expr>> &terms);

  void findNegatingTerms(ref<Expr> e,
                         std::vector<ref<Expr>> &terms);

  void findNegatingTerms(const ConstraintSet &constraints,
                         bool checkImplied,
                         bool checkConstraints,
                         std::vector<ref<Expr>> &terms);
};

void findAssertions(ref<ForallExpr> f,
                    std::vector<EqAssertion> &assertions);

void generateLemmaFromForall(ref<ForallExpr> f,
                             ConstraintSet &constraints,
                             bool checkImplied,
                             bool checkConstraints,
                             std::vector<ref<Expr>> &lemmas);

ref<Expr> findForallExpr(ref<Expr> e);

}

#endif
