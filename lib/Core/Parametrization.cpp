#include "Parametrization.h"
#include "ExecTreeIterator.h"

#include <klee/Expr/ArrayCache.h>
#include <klee/Expr/ExprUtil.h>
#include <klee/Support/ErrorHandling.h>

#include "llvm/ADT/StringExtras.h"

#define ATTEMPTS (2)

using namespace llvm;
using namespace klee;

static ArrayCache cache;

/* TODO: the size probably shouldn't be a parameter */
const Array *klee::getArray(const std::string &name,
                            uint64_t size,
                            bool isBoundVariable) {
  const Array *array = cache.CreateArray(name, size);
  if (isBoundVariable) {
    array->isBoundVariable = isBoundVariable;
  }
  return array;
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
      systems[i].push_back(SMTEquation(e, k + 1));
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

void klee::extractEquationsForValue(ExecTree &t,
                                    PatternMatch &pm,
                                    State2Value &valuesMap,
                                    SMTEquationSystem &system) {
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
    }

    uint32_t stateID = iter.getCurrent()->stateID;
    auto i = valuesMap.find(stateID);
    assert(i != valuesMap.end());
    system.push_back(SMTEquation(i->second, sm.count));
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

/* TODO: must pass both e1 and e2? */
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
        klee_message("multiple distinct children expressions");
        return nullptr;
      }

      ref<Expr> replacedKid = replaceDistinctTerms(k1, k2, placeHolder);
      if (replacedKid.isNull()) {
        return nullptr;
      }
      kids[i] = replacedKid;
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

ref<Expr> klee::getSymbolicValue(const Array *array, unsigned size) {
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
  case 1:
    return ConstantExpr::create(n, Expr::Int8);
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
                                uint32_t id,
                                ParametrizedExpr &templateExpr) {
  /* check width consistency */
  Expr::Width width;
  if (!checkWidthConsistency(constants, width)) {
    assert(0);
  }

  unsigned size = width / 8;

  const Array *array_a = getArray("a", QuantifiedExpr::AUX_VARIABLE_WIDTH);
  const Array *array_b = getArray("b", QuantifiedExpr::AUX_VARIABLE_WIDTH);
  const Array *array_m = getArray("m_" + llvm::utostr(id),
                                  QuantifiedExpr::AUX_VARIABLE_WIDTH);

  ref<Expr> a = getSymbolicValue(array_a, size);
  ref<Expr> b = getSymbolicValue(array_b, size);
  ref<Expr> m = getSymbolicValue(array_m, size);

  ref<Expr> all = ConstantExpr::create(1, Expr::Bool);
  for (unsigned i = 0; i < system.size(); i++) {
    /* c[i] = a * k[i] + b */
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

  /* must be empty */
  ConstraintSet s;

  /* first check is satisfiable */
  bool mayBeTrue;
  SolverQueryMetaData metaData;
  assert(solver.mayBeTrue(s, all, mayBeTrue, metaData, true));
  if (!mayBeTrue) {
    return false;
  }

  /* now, we can add the constraint as it is feasible */
  s.push_back(all);

  /* get assignment */
  std::vector<const Array *> objects = {array_a, array_b};
  std::vector<std::vector<unsigned char>> result;
  if (!solver.getInitialValues(s, objects, result, metaData, true)) {
    assert(0);
  }

  ref<Expr> coefficient_a = getConstantExpr(result[0], size);
  ref<Expr> coefficient_b = getConstantExpr(result[1], size);

  /* parameter: a * m + b */
  templateExpr.e = AddExpr::create(
    MulExpr::create(
      coefficient_a,
      m
    ),
    coefficient_b
  );
  templateExpr.parameter = m;
  templateExpr.array = array_m;

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
                               uint32_t id,
                               ParametrizedExpr &result) {
  assert(!system.empty());

  if (system.size() == 1) {
    /* no parameter here */
    klee_message("system has only one equation");
    result = ParametrizedExpr(system[0].e, nullptr, nullptr);
    return true;
  }

  SMTEquation eq1, eq2;
  ref<Expr> r1, r2;
  bool found = false;
  for (unsigned i = 0; i < ATTEMPTS; i++) {
    if (system.size() < i + 2) {
      /* not enough equations */
      klee_message("failed to find distinct terms after %u attempts", i);
      return false;
    }

    eq1 = system[i];
    eq2 = system[i + 1];
    found = findDistinctTerms(eq1.e, eq2.e, r1, r2);
    if (found) {
      break;
    }
  }

  if (!found) {
    klee_message("failed to find distinct terms");
    return false;
  }

  ParametrizedExpr templateExpr;
  if (!solveLinearEquation(solver, {eq1, eq2}, {r1, r2}, id, templateExpr)) {
    klee_message("failed to solve linear equation");
    return false;
  }

  ref<Expr> e = replaceDistinctTerms(eq1.e, eq2.e, templateExpr.e);
  if (e.isNull()) {
    return false;
  }

  ParametrizedExpr parametricExpr(e, templateExpr.parameter, templateExpr.array);
  if (validateSolution(system, parametricExpr)) {
    result = parametricExpr;
    return true;
  } else {
    klee_message("invalid solution");
    return false;
  }
}
