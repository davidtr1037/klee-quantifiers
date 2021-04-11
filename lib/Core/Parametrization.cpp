#include "Parametrization.h"
#include "ExecTreeIterator.h"

#include <klee/Expr/ArrayCache.h>

using namespace llvm;
using namespace klee;

static ArrayCache cache;

void klee::extractEquationsForCore(ExecTree &t,
                                   PatternMatch &pm,
                                   std::vector<SMTEquationSystem> &result) {
  std::vector<SMTEquationSystem> systems(pm.pattern.core.size());

  /* find max k */
  unsigned max_k = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max_k) {
      max_k = sm.count;
    }
  }

  ExecTreeIterator iter(t);

  /* traverse prefix */
  for (unsigned i = 0; i < pm.pattern.prefix.size(); i++) {
    assert(iter.hasNext());
    iter.next(pm.pattern.prefix[i]);
  }

  /* traverse core */
  for (unsigned k = 0; k < max_k; k++) {
    for (unsigned i = 0; i < pm.pattern.core.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.core[i]);

      ref<Expr> e = iter.getCurrent()->e;
      systems[i].push_back(SMTEquation(e, k));
    }
  }

  /* TODO: avoid copy */
  for (SMTEquationSystem &es : systems) {
    result.push_back(es);
  }
}

void klee::extractEquationsForSuffix(ExecTree &t,
                                     PatternMatch &pm,
                                     std::vector<SMTEquationSystem> &result) {
  std::vector<SMTEquationSystem> systems(pm.pattern.suffix.size());

  for (StateMatch &sm : pm.matches) {
    ExecTreeIterator iter(t);

    /* traverse prefix */
    for (unsigned i = 0; i < pm.pattern.prefix.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.prefix[i]);
    }

    /* traverse core */
    for (unsigned k = 0; k < sm.count; k++) {
      for (unsigned i = 0; i < pm.pattern.core.size(); i++) {
        assert(iter.hasNext());
        iter.next(pm.pattern.core[i]);
      }
    }

    /* traverse suffix */
    for (unsigned i = 0; i < pm.pattern.suffix.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.suffix[i]);

      ref<Expr> e = iter.getCurrent()->e;
      systems[i].push_back(SMTEquation(e, sm.count));
    }
  }

  /* TODO: avoid copy */
  for (SMTEquationSystem &es : systems) {
    result.push_back(es);
  }
}

static bool findDistinctTerms(ref<Expr> e1,
                              ref<Expr> e2,
                              ref<Expr> &r1,
                              ref<Expr> &r2) {
  if (*e1 == *e2) {
    return false;
  }

  if (e1->getKind() != e2->getKind()) {
    return false;
  }

  if (isa<ConstantExpr>(e1)) {
    r1 = e1;
    r2 = e2;
    return true;
  }

  if (isa<BinaryExpr>(e1)) {
    ref<BinaryExpr> t1 = dyn_cast<BinaryExpr>(e1);
    ref<BinaryExpr> t2 = dyn_cast<BinaryExpr>(e2);

    if (*t1->left != *t2->left) {
      return findDistinctTerms(t1->left, t2->left, r1, r2);
    }
    if (*t1->right != *t2->right) {
      return findDistinctTerms(t1->right, t2->right, r1, r2);
    }
  }
  else if (isa<NotOptimizedExpr>(e1)) {
    ref<NotOptimizedExpr> t1 = dyn_cast<NotOptimizedExpr>(e1);
    ref<NotOptimizedExpr> t2 = dyn_cast<NotOptimizedExpr>(e2);

    if (*t1->src != *t2->src) {
      return findDistinctTerms(t1->src, t2->src, r1, r2);
    }
  }
  else if (isa<ReadExpr>(e1)) {
    ref<ReadExpr> t1 = dyn_cast<ReadExpr>(e1);
    ref<ReadExpr> t2 = dyn_cast<ReadExpr>(e2);

    if (*t1->index != *t2->index) {
      return findDistinctTerms(t1->index, t2->index, r1, r2);
    }

    assert(t1->updates.root->isConstantArray());
    /* TODO: handle updates */
  }
  else if (isa<SelectExpr>(e1)) {
    ref<SelectExpr> t1 = dyn_cast<SelectExpr>(e1);
    ref<SelectExpr> t2 = dyn_cast<SelectExpr>(e2);

    if (*t1->cond != *t2->cond) {
      return findDistinctTerms(t1->cond, t2->cond, r1, r2);
    }
    if (*t1->trueExpr != *t2->trueExpr) {
      return findDistinctTerms(t1->trueExpr, t2->trueExpr, r1, r2);
    }
    if (*t1->falseExpr != *t2->falseExpr) {
      return findDistinctTerms(t1->falseExpr, t2->falseExpr, r1, r2);
    }
  }
  else if (isa<ConcatExpr>(e1)) {
    ref<ConcatExpr> t1 = dyn_cast<ConcatExpr>(e1);
    ref<ConcatExpr> t2 = dyn_cast<ConcatExpr>(e2);

    if (*t1->getLeft() != *t2->getLeft()) {
      return findDistinctTerms(t1->getLeft(), t2->getLeft(), r1, r2);
    }
    if (*t1->getRight() != *t2->getRight()) {
      return findDistinctTerms(t1->getRight(), t2->getRight(), r1, r2);
    }
  }
  else if (isa<ExtractExpr>(e1)) {
    ref<ExtractExpr> t1 = dyn_cast<ExtractExpr>(e1);
    ref<ExtractExpr> t2 = dyn_cast<ExtractExpr>(e2);

    if (*t1->expr != *t2->expr) {
      return findDistinctTerms(t1->expr, t2->expr, r1, r2);
    }
  }
  else if (isa<NotExpr>(e1)) {
    ref<NotExpr> t1 = dyn_cast<NotExpr>(e1);
    ref<NotExpr> t2 = dyn_cast<NotExpr>(e2);

    if (*t1->expr != *t2->expr) {
      return findDistinctTerms(t1->expr, t2->expr, r1, r2);
    }
  }
  else if (isa<CastExpr>(e1)) {
    ref<CastExpr> t1 = dyn_cast<CastExpr>(e1);
    ref<CastExpr> t2 = dyn_cast<CastExpr>(e2);

    if (*t1->src != *t2->src) {
      return findDistinctTerms(t1->src, t2->src, r1, r2);
    }
  } else {
    assert(0);
  }

  return false;
}

bool checkWidthConsistency(const std::vector<ref<Expr>> &constants,
                           Expr::Width &w) {
  w = 0;
  for (const ref<Expr> &e : constants) {
    if (w == 0) {
      w = e->getWidth();
    } else {
      if (e->getWidth() != w) {
        return false;
      }
    }
  }

  return true;
}

static ref<Expr> getSymbolicValue(const Array *array, unsigned size) {
  ref<Expr> r = nullptr;
  for (unsigned i = 0; i < size; i++) {
    ref<Expr> b = ReadExpr::create(UpdateList(array, 0),
                                   ConstantExpr::alloc(i, Expr::Int32));
    if (r.isNull()) {
      r = b;
    } else {
      r = ConcatExpr::create(b, r);
    }
  }
  return r;
}

static ref<Expr> getConstantExpr(std::vector<unsigned char> &v) {
  ref<Expr> r = nullptr;
  uint64_t n = 0;
  for (unsigned i = 0; i < v.size(); i++) {
    n |= ((uint64_t)(v[i]) << (i * 8));
  }

  if (v.size() == 4) {
    return ConstantExpr::create(n, Expr::Int32);
  }
  else if (v.size() == 8) {
    return ConstantExpr::create(n, Expr::Int64);
  } else {
    assert(0);
    return nullptr;
  }
}

static bool solveLinearEquation(TimingSolver &solver,
                                const SMTEquationSystem &system,
                                const std::vector<ref<Expr>> &constants,
                                ParametrizedExpr &pe) {
  /* check width consistency */
  Expr::Width width;
  if (!checkWidthConsistency(constants, width)) {
    assert(0);
  }

  const Array *array_a = cache.CreateArray("a", width / 8);
  const Array *array_b = cache.CreateArray("b", width / 8);
  const Array *array_m = cache.CreateArray("m", width / 8);

  ref<Expr> a = getSymbolicValue(array_a, width / 8);
  ref<Expr> b = getSymbolicValue(array_b, width / 8);
  ref<Expr> m = getSymbolicValue(array_m, width / 8);

  ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
  for (unsigned i = 0; i < system.size(); i++) {
    ref<Expr> eq = EqExpr::create(
      constants[i],
      AddExpr::create(
        MulExpr::create(
          a,
          ConstantExpr::create(system[i].k, a->getWidth())
        ),
        b
      )
    );
    all = AndExpr::create(all, eq);
  }

  ConstraintSet s({all});

  /* first check is satisfiable */
  bool mayBeTrue;
  SolverQueryMetaData metaData;
  assert(solver.mayBeTrue(s, all, mayBeTrue, metaData));

  /* get assignment */
  std::vector<const Array *> objects = {array_a, array_b};
  std::vector<std::vector<unsigned char>> result;
  assert(solver.getInitialValues(s, objects, result, metaData));

  ref<Expr> coefficient_a = getConstantExpr(result[0]);
  ref<Expr> coefficient_b = getConstantExpr(result[1]);

  pe.e = AddExpr::create(
    MulExpr::create(
      coefficient_a,
      m
    ),
    coefficient_b
  );
  pe.parameter = m;

  return true;
}

static bool validateSolution(const SMTEquationSystem &system,
                             ParametrizedExpr &pe) {
  return true;
}

bool klee::solveEquationSystem(SMTEquationSystem &system,
                               TimingSolver &solver,
                               ParametrizedExpr &solution) {
  assert(system.size() >= 2);

  /* TODO: ... */
  SMTEquation eq1 = system[0];
  SMTEquation eq2 = system[1];

  ref<Expr> r1, r2;
  if (!findDistinctTerms(eq1.e, eq2.e, r1, r2)) {
    assert(0);
  }

  ParametrizedExpr pe;
  solveLinearEquation(solver, {eq1, eq2}, {r1, r2}, pe);
  validateSolution(system, pe);

  return true;
}
