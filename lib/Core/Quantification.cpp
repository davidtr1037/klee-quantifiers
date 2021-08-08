#include "Quantification.h"
#include "klee/Expr/ExprUtil.h"

#include "llvm/ADT/StringExtras.h"

using namespace klee;
using namespace llvm;

bool getParametricExpressions(std::vector<SMTEquationSystem> systems,
                              TimingSolver &solver,
                              uint32_t mergeID,
                              std::vector<ParametrizedExpr> &result) {
  for (SMTEquationSystem &system : systems) {
    ParametrizedExpr solution;
    if (!solveEquationSystem(system, solver, mergeID, solution)) {
      return false;
    }

    result.push_back(solution);
  }

  return true;
}

ref<Expr> generateForallPremise(ref<Expr> bound, ref<Expr> parameter) {
  return AndExpr::create(
    UleExpr::create(ConstantExpr::create(1, parameter->getWidth()), bound),
    UleExpr::create(bound, parameter)
  );
}

ref<Expr> generateRangeConstraint(PatternMatch &pm, ref<Expr> aux) {
  unsigned min = -1, max = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max) {
      max = sm.count;
    }
    if (sm.count <= min) {
      min = sm.count;
    }
  }

  ref<Expr> maxExpr = ConstantExpr::create(max, aux->getWidth());
  ref<Expr> minExpr = ConstantExpr::create(min, aux->getWidth());

  if (max == min) {
    return EqExpr::create(aux, maxExpr);
  }

  /* TODO: handle missing values */
  assert((max - min + 1) == pm.matches.size());
  return AndExpr::create(
    UgeExpr::create(aux, minExpr),
    UleExpr::create(aux, maxExpr)
  );
}

void generateForall(PatternMatch &pm,
                    std::vector<ParametrizedExpr> solutions,
                    uint32_t mergeID,
                    ref<Expr> &forallExpr,
                    ref<Expr> &rangeExpr) {
  if (solutions.empty()) {
    forallExpr = rangeExpr = ConstantExpr::create(1, Expr::Bool);
    return;
  }

  unsigned auxArraySize = QuantifiedExpr::AUX_VARIABLE_WIDTH / 8;
  const Array *array_i = getArray("__i", auxArraySize, true, false);
  /* TODO: reuse the array from other module */
  const Array *array_m = getArray("m_" + llvm::utostr(mergeID),
                                  auxArraySize,
                                  false,
                                  true);

  /* the introduced variable */
  ref<Expr> aux = getSymbolicValue(array_m, array_m->size);
  /* outside of the forall expression */
  rangeExpr = generateRangeConstraint(pm, aux);

  ref<Expr> bound = getSymbolicValue(array_i, array_i->size);
  ref<Expr> premise = generateForallPremise(bound, aux);

  ref<Expr> coreExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : solutions) {
    /* TODO: rename */
    ref<Expr> i = getSymbolicValue(array_i, pe.parameter->getWidth() / 8);
    ExprReplaceVisitor visitor(pe.parameter, i);
    /* TODO: rename */
    ref<Expr> substituted = visitor.visit(pe.e);
    coreExpr = AndExpr::create(coreExpr, substituted);
  }

  /* the forall expression itself */
  forallExpr = ForallExpr::create(
    bound,
    premise,
    coreExpr
  );
}


/* TODO: handle low number of matches */
ref<Expr> klee::generateQuantifiedConstraint(PatternMatch &pm,
                                             ExecTree &tree,
                                             uint32_t mergeID,
                                             TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  ref<Expr> prefix = extractPrefixConstraint(tree, pm);

  extractEquationsForCore(tree, pm, coreSystems);
  if (!getParametricExpressions(coreSystems, solver, mergeID, coreSolutions)) {
    return nullptr;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (!getParametricExpressions(suffixSystems, solver, mergeID, suffixSolutions)) {
    return nullptr;
  }

  ref<Expr> forallExpr, rangeExpr;
  generateForall(pm, coreSolutions, mergeID, forallExpr, rangeExpr);

  ref<Expr> suffixExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : suffixSolutions) {
    suffixExpr = AndExpr::create(suffixExpr, pe.e);
  }

  //errs() << "prefix:\n";
  //prefix->dump();
  //errs() << "core premise:\n";
  //premise->dump();
  //errs() << "core:\n";
  //for (const ParametrizedExpr &pe : coreSolutions) {
  //  pe.e->dump();
  //}
  //errs() << "suffix:\n";
  //for (const ParametrizedExpr &pe : suffixSolutions) {
  //  pe.e->dump();
  //}
  //errs() << "range:\n";
  //rangeExpr->dump();

  return AndExpr::create(
    prefix,
    AndExpr::create(
      rangeExpr,
      AndExpr::create(
        forallExpr,
        suffixExpr
      )
    )
  );
}

bool klee::generateMergedValue(PatternMatch &pm,
                               ExecTree &tree,
                               State2Value &valuesMap,
                               uint32_t mergeID,
                               TimingSolver &solver,
                               ParametrizedExpr &solution) {
  SMTEquationSystem system;
  extractEquationsForValue(tree, pm, valuesMap, system);

  if (!solveEquationSystem(system, solver, mergeID, solution)) {
    return false;
  }

  return true;
}
