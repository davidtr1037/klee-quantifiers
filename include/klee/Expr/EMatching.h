#ifndef KLEE_EMATCHING_H
#define KLEE_EMATCHING_H

#include "klee/ADT/Ref.h"
#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"

namespace klee {

class BaseAssertion {

public:

  class ReferenceCounter _refCount;

  BaseAssertion() {

  }

  virtual ~BaseAssertion() {

  }

  virtual void findImpliedNegatingTerms(std::vector<ref<Expr>> &terms) {

  }

  virtual void findNegatingTerms(ref<Expr> e,
                                 std::vector<ref<Expr>> &terms) {

  }

  void findNegatingTerms(const ConstraintSet &constraints,
                         bool checkImplied,
                         bool checkConstraints,
                         std::vector<ref<Expr>> &terms);

  void intersect(const std::vector<std::vector<ref<Expr>>> &groups,
                 std::vector<ref<Expr>> &result);
};

class EqAssertion : public BaseAssertion {

public:

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
};

class AndAssertion : public BaseAssertion {

public:

  std::vector<ref<BaseAssertion>> assertions;

  AndAssertion(const std::vector<ref<BaseAssertion>> &assertions);

  void findImpliedNegatingTerms(std::vector<ref<Expr>> &terms);

  void findNegatingTerms(ref<Expr> e,
                         std::vector<ref<Expr>> &terms);
};

class OrAssertion : public BaseAssertion {

public:

  std::vector<ref<BaseAssertion>> assertions;

  OrAssertion(const std::vector<ref<BaseAssertion>> &assertions);

  void findImpliedNegatingTerms(std::vector<ref<Expr>> &terms);

  void findNegatingTerms(ref<Expr> e,
                         std::vector<ref<Expr>> &terms);
};

void findAssertions(ref<ForallExpr> f,
                    std::vector<ref<BaseAssertion>> &assertions);

void generateLemmaFromForall(ref<ForallExpr> f,
                             ConstraintSet &constraints,
                             bool checkImplied,
                             bool checkConstraints,
                             std::vector<ref<Expr>> &lemmas);

ref<Expr> findForallExpr(ref<Expr> e);

}

#endif
