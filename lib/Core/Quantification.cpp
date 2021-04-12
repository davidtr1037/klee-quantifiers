#include "Quantification.h"

using namespace klee;
using namespace llvm;

bool getParametricExpressions(std::vector<SMTEquationSystem> systems,
                              TimingSolver &solver,
                              std::vector<ParametrizedExpr> &result) {
  for (SMTEquationSystem &system : systems) {
    ParametrizedExpr solution;
    if (!solveEquationSystem(system, solver, solution)) {
      return false;
    }

    result.push_back(solution);
  }

  return true;
}

ref<Expr> generateRangeConstraint(PatternMatch &pm, ref<Expr> parameter) {
  unsigned min = -1, max = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max) {
      max = sm.count;
    }
    if (sm.count <= min) {
      min = sm.count;
    }
  }

  ref<Expr> maxExpr = ConstantExpr::create(max, parameter->getWidth());
  ref<Expr> minExpr = ConstantExpr::create(min, parameter->getWidth());
  if (max == min) {
    return EqExpr::create(parameter, maxExpr);
  }

  /* TODO: handle missing values */
  assert((max - min + 1) == pm.matches.size());
  return AndExpr::create(
    UgeExpr::create(parameter, minExpr),
    UleExpr::create(parameter, maxExpr)
  );
}

void klee::generateQuantifiedConstraint(PatternMatch &pm,
                                        ExecTree &tree,
                                        TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  ref<Expr> prefix = extractPrefixConstraint(tree, pm);

  extractEquationsForCore(tree, pm, coreSystems);
  if (!getParametricExpressions(coreSystems, solver, coreSolutions)) {
    return;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (!getParametricExpressions(suffixSystems, solver, suffixSolutions)) {
    return;
  }

  /* TODO: check parameter consistency */
  assert(!coreSolutions.empty());
  ref<Expr> parameter = coreSolutions[0].parameter;
  ref<Expr> rangeExpr = generateRangeConstraint(pm, parameter);

  errs() << "prefix:\n";
  prefix->dump();
  errs() << "core:\n";
  for (const ParametrizedExpr &pe : coreSolutions) {
    pe.e->dump();
  }
  errs() << "suffix:\n";
  for (const ParametrizedExpr &pe : suffixSolutions) {
    pe.e->dump();
  }
  errs() << "range:\n";
  rangeExpr->dump();
}
