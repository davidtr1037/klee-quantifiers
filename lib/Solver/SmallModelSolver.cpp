#include "SmallModelSolver.h"
#include "klee/Solver/Solver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"

using namespace llvm;
using namespace klee;

SmallModelSolver::SmallModelSolver(Solver *solver) : solver(solver) {

}

SmallModelSolver::~SmallModelSolver() {
  delete solver;
}

bool SmallModelSolver::computeTruth(const Query& query,
                                    bool &isValid) {
  return solver->impl->computeTruth(query, isValid);
}

bool SmallModelSolver::computeValidity(const Query& query,
                                       Solver::Validity &result) {
  return solver->impl->computeValidity(query, result);
}

bool SmallModelSolver::computeValue(const Query& query,
                                    ref<Expr> &result) {
  return solver->impl->computeValue(query, result);
}

bool SmallModelSolver::computeInitialValues(const Query& query,
                                            const std::vector<const Array*> &objects,
                                            std::vector< std::vector<unsigned char> > &values,
                                            bool &hasSolution) {
  return solver->impl->computeInitialValues(query,
                                            objects,
                                            values,
                                            hasSolution);
}

SolverImpl::SolverRunStatus SmallModelSolver::getOperationStatusCode() {
  return solver->impl->getOperationStatusCode();
}

char *SmallModelSolver::getConstraintLog(const Query& query) {
  return solver->impl->getConstraintLog(query);
}

void SmallModelSolver::setCoreSolverTimeout(time::Span timeout) {
  solver->impl->setCoreSolverTimeout(timeout);
}

Solver *klee::createSmallModelSolver(Solver *solver) {
  return new Solver(new SmallModelSolver(solver));
}
