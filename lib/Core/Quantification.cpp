#include "Quantification.h"
#include "klee/Expr/ExprUtil.h"

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

ref<Expr> generateForallPremise(ref<Expr> bound, ref<Expr> parameter) {
  return AndExpr::create(
    UleExpr::create(ConstantExpr::create(1, parameter->getWidth()), bound),
    UleExpr::create(bound, parameter)
  );
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

ref<Expr> klee::generateQuantifiedConstraint(PatternMatch &pm,
                                             ExecTree &tree,
                                             TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  ref<Expr> prefix = extractPrefixConstraint(tree, pm);

  extractEquationsForCore(tree, pm, coreSystems);
  if (!getParametricExpressions(coreSystems, solver, coreSolutions)) {
    return nullptr;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (!getParametricExpressions(suffixSystems, solver, suffixSolutions)) {
    return nullptr;
  }

  const Array *array_i = getArray("__i", 8);

  ref<Expr> coreExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : coreSolutions) {
    ref<Expr> i = getSymbolicValue(array_i, pe.parameter->getWidth() / 8);
    ExprReplaceVisitor visitor(pe.parameter, i);
    ref<Expr> substituted = visitor.visit(pe.e);
    coreExpr = AndExpr::create(coreExpr, substituted);
  }

  ref<Expr> suffixExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : suffixSolutions) {
    suffixExpr = AndExpr::create(suffixExpr, pe.e);
  }

  /* TODO: check parameter consistency */
  assert(!coreSolutions.empty());
  ref<Expr> parameter = coreSolutions[0].parameter;
  ref<Expr> bound = getSymbolicValue(array_i, parameter->getWidth() / 8);
  ref<Expr> premise = generateForallPremise(bound, parameter);
  ref<Expr> rangeExpr = generateRangeConstraint(pm, parameter);

  errs() << "prefix:\n";
  prefix->dump();
  errs() << "core premise:\n";
  premise->dump();
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

  return AndExpr::create(
    prefix,
    AndExpr::create(
      rangeExpr,
      AndExpr::create(
        ForallExpr::create(
          bound,
          OrExpr::create(
            Expr::createIsZero(premise),
            coreExpr
          )
        ),
        suffixExpr
      )
    )
  );
}
