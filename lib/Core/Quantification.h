#ifndef KLEE_QUANTIFICATION_H
#define KLEE_QUANTIFICATION_H

#include "TimingSolver.h"
#include "PatternExtraction.h"
#include "Parametrization.h"

namespace klee {

ref<Expr> generateQuantifiedConstraint(PatternMatch &pm,
                                       ExecTree &tree,
                                       uint32_t mergeID,
                                       TimingSolver &solver);

bool generateMergedValue(PatternMatch &pm,
                         ExecTree &tree,
                         State2Value &valuesMap,
                         uint32_t mergeID,
                         TimingSolver &solver,
                         ParametrizedExpr &solution);

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

void generateLemmaFromForall(ref<ForallExpr> f,
                             ConstraintSet &constraints,
                             bool checkImplied,
                             bool checkConstraints,
                             std::vector<ref<Expr>> &lemmas);

ref<Expr> findForallExpr(ref<Expr> e);

}

#endif
