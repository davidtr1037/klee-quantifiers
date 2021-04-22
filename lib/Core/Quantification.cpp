#include "Quantification.h"
#include "klee/Expr/ExprUtil.h"

using namespace klee;
using namespace llvm;

bool getParametricExpressions(std::vector<SMTEquationSystem> systems,
                              TimingSolver &solver,
                              uint32_t id,
                              std::vector<ParametrizedExpr> &result) {
  for (SMTEquationSystem &system : systems) {
    ParametrizedExpr solution;
    if (!solveEquationSystem(system, solver, id, solution)) {
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

ref<Expr> generateRangeConstraint(PatternMatch &pm, const Array *array) {
  unsigned min = -1, max = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max) {
      max = sm.count;
    }
    if (sm.count <= min) {
      min = sm.count;
    }
  }

  Expr::Width w = QuantifiedExpr::AUX_VARIABLE_WIDTH;
  ref<Expr> maxExpr = ConstantExpr::create(max, w);
  ref<Expr> minExpr = ConstantExpr::create(min, w);
  ref<Expr> parameter = getSymbolicValue(array, w / 8);

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

void generateForall(PatternMatch &pm,
                    std::vector<ParametrizedExpr> solutions,
                    ref<Expr> &forallExpr,
                    ref<Expr> &rangeExpr) {
  if (solutions.empty()) {
    forallExpr = rangeExpr = ConstantExpr::create(1, Expr::Bool);
    return;
  }

  const Array *array_i = getArray("__i", 8, true);

  ref<Expr> coreExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : solutions) {
    ref<Expr> i = getSymbolicValue(array_i, pe.parameter->getWidth() / 8);
    ExprReplaceVisitor visitor(pe.parameter, i);
    ref<Expr> substituted = visitor.visit(pe.e);
    coreExpr = AndExpr::create(coreExpr, substituted);
  }

  /* TODO: not so elegant */
  ParametrizedExpr &pe = solutions[0];
  /* TODO: rename parameter --> aux */
  ref<Expr> parameter = pe.parameter;

  /* outside of the forall expression */
  rangeExpr = generateRangeConstraint(pm, pe.array);

  /* the forall expression itself */
  ref<Expr> bound = getSymbolicValue(array_i, parameter->getWidth() / 8);
  ref<Expr> premise = generateForallPremise(bound, parameter);
  forallExpr = ForallExpr::create(
    bound,
    OrExpr::create(
      Expr::createIsZero(premise),
      coreExpr
    )
  );
}


/* TODO: handle low number of matches */
ref<Expr> klee::generateQuantifiedConstraint(PatternMatch &pm,
                                             ExecTree &tree,
                                             uint32_t id,
                                             TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  ref<Expr> prefix = extractPrefixConstraint(tree, pm);

  extractEquationsForCore(tree, pm, coreSystems);
  if (!getParametricExpressions(coreSystems, solver, id, coreSolutions)) {
    return nullptr;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (!getParametricExpressions(suffixSystems, solver, id, suffixSolutions)) {
    return nullptr;
  }

  ref<Expr> forallExpr, rangeExpr;
  generateForall(pm, coreSolutions, forallExpr, rangeExpr);

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
