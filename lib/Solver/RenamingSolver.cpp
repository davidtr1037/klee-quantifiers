#include "RenamingSolver.h"
#include "klee/Solver/Solver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprRename.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"

using namespace llvm;
using namespace klee;

RenamingSolver::RenamingSolver(Solver *solver) : solver(solver) {

}

RenamingSolver::~RenamingSolver() {
  delete solver;
}

bool RenamingSolver::computeTruth(const Query& query,
                                  bool &isValid) {
  ConstraintSet constraints;
  ref<Expr> expr;
  rename(query, constraints, expr);
  Query renamed(constraints, expr);
  return solver->impl->computeTruth(renamed, isValid);
}

bool RenamingSolver::computeValidity(const Query& query,
                                     Solver::Validity &result) {
  ConstraintSet constraints;
  ref<Expr> expr;
  rename(query, constraints, expr);
  Query renamed(constraints, expr);
  return solver->impl->computeValidity(renamed, result);
}

bool RenamingSolver::computeValue(const Query& query, ref<Expr> &result) {
  ConstraintSet constraints;
  ref<Expr> expr;
  rename(query, constraints, expr);
  Query renamed(constraints, expr);
  return solver->impl->computeValue(renamed, result);
}

bool RenamingSolver::computeInitialValues(const Query& query,
                                          const std::vector<const Array*> &objects,
                                          std::vector< std::vector<unsigned char> > &values,
                                          bool &hasSolution) {
  ConstraintSet constraints;
  ref<Expr> expr;
  std::vector<const Array *> renamedObjects;
  rename(query, objects, constraints, expr, renamedObjects);
  Query renamed(constraints, expr);
  return solver->impl->computeInitialValues(renamed,
                                            renamedObjects,
                                            values,
                                            hasSolution);
}

SolverImpl::SolverRunStatus RenamingSolver::getOperationStatusCode() {
  return solver->impl->getOperationStatusCode();
}

char *RenamingSolver::getConstraintLog(const Query& query) {
  return solver->impl->getConstraintLog(query);
}

void RenamingSolver::setCoreSolverTimeout(time::Span timeout) {
  solver->impl->setCoreSolverTimeout(timeout);
}

Solver *klee::createRenamingSolver(Solver *solver) {
  return new Solver(new RenamingSolver(solver));
}
