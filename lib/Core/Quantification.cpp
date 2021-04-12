#include "Quantification.h"

using namespace klee;

void klee::generateQuantifiedConstraint(const PatternMatch &pm) {
  std::vector<SMTEquationSystem> systems;
  extractEquationsForCore(loopHandler->tree, pm, systems);
  extractEquationsForSuffix(loopHandler->tree, pm, systems);

  for (SMTEquationSystem &system : systems) {
    ParametrizedExpr solution;
    solveEquationSystem(system,
                        *loopHandler->solver,
                        solution);
  }
}
