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

void klee::generateQuantifiedConstraint(PatternMatch &pm,
                                        ExecTree &tree,
                                        TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  extractEquationsForCore(tree, pm, coreSystems);
  if (getParametricExpressions(coreSystems, solver, coreSolutions)) {
    return;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (getParametricExpressions(suffixSystems, solver, suffixSolutions)) {
    return;
  }

  errs() << "core:\n";
  for (const ParametrizedExpr &pe : coreSolutions) {
    pe.e->dump();
  }
  errs() << "suffix:\n";
  for (const ParametrizedExpr &pe : suffixSolutions) {
    pe.e->dump();
  }
}
