#include "Parametrization.h"
#include "ExecTreeIterator.h"

#include <klee/Expr/ArrayCache.h>
#include <klee/Expr/ExprUtil.h>

using namespace llvm;
using namespace klee;

static ArrayCache cache;

const Array *klee::getArray(const std::string &name, uint64_t size) {
  return cache.CreateArray(name, size);
}

ref<Expr> klee::extractPrefixConstraint(ExecTree &t,
                                        PatternMatch &pm) {
  ExecTreeIterator iter(t);

  ref<Expr> prefix = ConstantExpr::create(1, Expr::Bool);

  /* traverse prefix */
  for (unsigned i = 0; i < pm.pattern.prefix.size(); i++) {
    assert(iter.hasNext());
    iter.next(pm.pattern.prefix[i]);
    prefix = AndExpr::create(prefix, iter.getCurrent()->e);
  }

  return prefix;
}

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

  /* TODO: if we have a read expression, make sure it has a constant array */
  for (unsigned i = 0; i < e1->getNumKids(); i++) {
    ref<Expr> k1 = e1->getKid(i);
    ref<Expr> k2 = e2->getKid(i);
    if (*k1 != *k2) {
      return findDistinctTerms(k1, k2, r1, r2);
    }
  }

  return false;
}

static ref<Expr> replaceDistinctTerms(ref<Expr> e1,
                                      ref<Expr> e2,
                                      ref<Expr> placeHolder) {
  if (*e1 == *e2) {
    assert(false);
  }

  if (e1->getKind() != e2->getKind()) {
    assert(false);
  }

  if (isa<ConstantExpr>(e1)) {
    assert(placeHolder->getWidth() == e1->getWidth());
    return placeHolder;
  }

  bool replaced = false;
  ref<Expr> kids[8];
  for (unsigned i = 0; i < e1->getNumKids(); i++) {
    ref<Expr> k1 = e1->getKid(i);
    ref<Expr> k2 = e2->getKid(i);
    if (*k1 == *k2) {
      kids[i] = k1;
    } else {
      if (replaced) {
        assert(false);
      }

      kids[i] = replaceDistinctTerms(k1, k2, placeHolder);
      replaced = true;
    }
  }

  return e1->rebuild(kids);
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

static ref<Expr> getConstantExpr(std::vector<unsigned char> &v,
                                 unsigned size) {
  assert(v.size() >= size);

  ref<Expr> r = nullptr;
  uint64_t n = 0;
  for (unsigned i = 0; i < size; i++) {
    n |= ((uint64_t)(v[i]) << (i * 8));
  }

  switch (size) {
  case 4:
    return ConstantExpr::create(n, Expr::Int32);
  case 8:
    return ConstantExpr::create(n, Expr::Int64);
  default:
    assert(0);
    return nullptr;
  }
}

static bool solveLinearEquation(TimingSolver &solver,
                                const SMTEquationSystem &system,
                                const std::vector<ref<Expr>> &constants,
                                ParametrizedExpr &templateExpr) {
  /* check width consistency */
  Expr::Width width;
  if (!checkWidthConsistency(constants, width)) {
    assert(0);
  }

  /* TODO: use pointer width */
  unsigned size = width / 8;
  const Array *array_a = getArray("a", 8);
  const Array *array_b = getArray("b", 8);
  const Array *array_m = getArray("m", 8);

  ref<Expr> a = getSymbolicValue(array_a, size);
  ref<Expr> b = getSymbolicValue(array_b, size);
  ref<Expr> m = getSymbolicValue(array_m, size);

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

  ref<Expr> coefficient_a = getConstantExpr(result[0], size);
  ref<Expr> coefficient_b = getConstantExpr(result[1], size);

  templateExpr.e = AddExpr::create(
    MulExpr::create(
      coefficient_a,
      m
    ),
    coefficient_b
  );
  templateExpr.parameter = m;

  return true;
}

static bool validateSolution(const SMTEquationSystem &system,
                             const ParametrizedExpr &pe) {
  for (const SMTEquation &eq : system) {
    ref<Expr> from = pe.parameter;
    ref<ConstantExpr> to = ConstantExpr::create(eq.k, pe.parameter->getWidth());
    ExprReplaceVisitor visitor(from, to);
    ref<Expr> substituted = visitor.visit(pe.e);
    if (*substituted != *eq.e) {
      return false;
    }
  }

  return true;
}

bool klee::solveEquationSystem(SMTEquationSystem &system,
                               TimingSolver &solver,
                               ParametrizedExpr &result) {
  assert(system.size() >= 2);

  /* TODO: ... */
  SMTEquation eq1 = system[0];
  SMTEquation eq2 = system[1];

  ref<Expr> r1, r2;
  if (!findDistinctTerms(eq1.e, eq2.e, r1, r2)) {
    assert(0);
  }

  ParametrizedExpr templateExpr;
  solveLinearEquation(solver, {eq1, eq2}, {r1, r2}, templateExpr);

  ref<Expr> e = replaceDistinctTerms(eq1.e, eq2.e, templateExpr.e);
  ParametrizedExpr parametricExpr(e, templateExpr.parameter);
  if (validateSolution(system, parametricExpr)) {
    result = parametricExpr;
    return true;
  } else {
    return false;
  }
}
