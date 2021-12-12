#include "SmallModelSolver.h"
#include "klee/Solver/Solver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/Assignment.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"
#include "klee/Support/OptionCategories.h"

using namespace llvm;
using namespace klee;

cl::opt<bool> ValidateSmallModel(
  "validate-small-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

void SmallModelSolver::dumpModel(const std::vector<const Array *> &objects,
                                 const std::vector<std::vector<unsigned char>> &values) {
  assert(objects.size() == values.size());
  for (unsigned i = 0; i < objects.size(); i++) {
    const Array *array = objects[i];
    std::vector<unsigned char> bytes = values[i];
    errs() << array->getName() << "\n";
    for (unsigned j = 0; j < bytes.size(); j++) {
      errs() << (unsigned)(bytes[j]) << " ";
    }
    errs() << "\n";
  }
}

bool SmallModelSolver::evalModel(const Query &query,
                                 Assignment &assignment) {
  ref<Expr> e = assignment.evaluate(Expr::createIsZero(query.expr));
  if (e->isFalse()) {
    return false;
  }

  for (ref<Expr> constraint : query.constraints) {
    ref<Expr> e = assignment.evaluate(constraint);
    if (e->isFalse()) {
      return false;
    }
  }

  return true;
}

ref<Expr> SmallModelSolver::eliminateForall(ref<ForallExpr> f) {
  ref<Expr> e = instantiateForall(f, 1);
  ref<Expr> aux = getSymbolicValue(f->auxArray,
                                   QuantifiedExpr::AUX_VARIABLE_SIZE);
  return OrExpr::create(
    EqExpr::create(aux, ConstantExpr::create(0, aux->getWidth())),
    e
  );
}

ref<Expr> SmallModelSolver::transform(ref<Expr> e) {
  if (isa<ForallExpr>(e)) {
    return eliminateForall(dyn_cast<ForallExpr>(e));
  } else {
    return e;
  }
}

void SmallModelSolver::findAuxReads(ref<Expr> e,
                                    std::vector<ref<Expr>> &result) {
  if (e->hasAuxVariable) {
    std::vector<ref<ReadExpr>> reads;
    findReads(e, true, reads);
    for (ref<ReadExpr> r : reads) {
      if (r->index->hasAuxVariable) {
        /* TODO: is correct? */
        ref<Expr> index = r->index;
        ref<Expr> size = r->updates.root->sizeExpr;

        if (size->getWidth() < r->index->getWidth()) {
          size = ZExtExpr::create(size, index->getWidth());
        }
        if (size->getWidth() > r->index->getWidth()) {
          index = ZExtExpr::create(index, size->getWidth());
        }

        result.push_back(UltExpr::create(index, size));
      }
    }
  }
}

void SmallModelSolver::transform(const Query &query,
                                 ConstraintSet &constraints) {
  for (ref<Expr> e : query.constraints) {
    constraints.push_back(transform(e));
  }

  /* TODO: rename */
  std::vector<ref<Expr>> additional;
  for (ref<Expr> e : query.constraints) {
    findAuxReads(e, additional);
  }
  findAuxReads(query.expr, additional);

  for (ref<Expr> e : additional) {
    constraints.push_back(e);
  }
}

bool SmallModelSolver::getAccessedOffset(ref<Expr> e,
                                         uint64_t value,
                                         uint64_t &offset,
                                         const Array *&array) {
  std::vector<ref<ReadExpr>> reads;
  findReads(e, true, reads);
  for (ref<ReadExpr> r : reads) {
    if (r->index->hasBoundVariable) {
      ref<Expr> index = substBoundVariables(r->index, value);
      if (isa<ConstantExpr>(index)) {
        offset = dyn_cast<ConstantExpr>(index)->getZExtValue();
        array = r->updates.root;
        return true;
      }
    }
  }

  return false;
}

/* TODO: optimize */
char SmallModelSolver::getModelValue(const std::vector<const Array *> &objects,
                                     std::vector<std::vector<unsigned char>> &values,
                                     const Array *object,
                                     unsigned index) {
  for (unsigned i = 0; i < objects.size(); i++) {
    const Array *array = objects[i];
    if (array == object) {
      assert(index < values[i].size());
      return values[i][index];
    }
  }

  assert(0);
  return 0;
}

/* TODO: optimize */
void SmallModelSolver::setModelValue(const std::vector<const Array *> &objects,
                                     std::vector<std::vector<unsigned char>> &values,
                                     const Array *object,
                                     unsigned index,
                                     char value) {
  for (unsigned i = 0; i < objects.size(); i++) {
    const Array *array = objects[i];
    if (array == object) {
      assert(index < values[i].size());
      values[i][index] = value;
      return;
    }
  }

  assert(0);
}

bool SmallModelSolver::adjustModel(const Query &query,
                                   const std::vector<const Array *> &objects,
                                   std::vector<std::vector<unsigned char>> &values) {
  Assignment assignment(objects, values, true);
  for (ref<Expr> e : query.constraints) {
    if (isa<ForallExpr>(e)) {
      ref<ForallExpr> f = dyn_cast<ForallExpr>(e);
      ref<Expr> body = assignment.evaluate(f->post);

      uint64_t offset = 0;
      const Array *array = nullptr;
      if (!getAccessedOffset(body, 1, offset, array)) {
        return false;
      }

      char c = getModelValue(objects, values, array, offset);
      ref<Expr> aux = getSymbolicValue(f->auxArray, QuantifiedExpr::AUX_VARIABLE_SIZE);
      ref<ConstantExpr> substAux = dyn_cast<ConstantExpr>(assignment.evaluate(aux));
      assert(!substAux.isNull());
      uint64_t m = substAux->getZExtValue();
      for (uint64_t v = 2; v <= m; v++) {
        if (!getAccessedOffset(body, v, offset, array)) {
          return false;
        }
        setModelValue(objects, values, array, offset, c);
      }
    }
  }

  Assignment adjusted(objects, values, true);
  return evalModel(query, adjusted);
}

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

bool SmallModelSolver::computeInitialValuesUsingSmallModel(const Query &query,
                                                           const std::vector<const Array *> &objects,
                                                           std::vector<std::vector<unsigned char>> &values,
                                                           bool &hasSolution) {
  ConstraintSet constraints;
  transform(query, constraints);
  /* TODO: rename */
  Query smQuery(constraints, transform(query.expr));

  bool hasSmallModelSolution;
  bool success = solver->impl->computeInitialValues(smQuery,
                                                    objects,
                                                    values,
                                                    hasSmallModelSolution);
  if (!success) {
    return false;
  }

  if (hasSmallModelSolution) {
    if (!adjustModel(query, objects, values)) {
      return false;
    }
    hasSolution = true;
  } else {
    hasSolution = false;
  }

  return true;
}

bool SmallModelSolver::computeInitialValues(const Query& query,
                                            const std::vector<const Array*> &objects,
                                            std::vector<std::vector<unsigned char>> &values,
                                            bool &hasSolution) {
  bool success = computeInitialValuesUsingSmallModel(query,
                                                     objects,
                                                     values,
                                                     hasSolution);
  if (success) {
    if (ValidateSmallModel) {
      std::vector<std::vector<unsigned char>> values;
      bool expected;
      assert(solver->impl->computeInitialValues(query,
                                                objects,
                                                values,
                                                expected));
      assert(hasSolution == expected);
    }
    return true;
  }

  /* TODO: ugly... */
  values.clear();
  success = solver->impl->computeInitialValues(query,
                                               objects,
                                               values,
                                               hasSolution);
  return success;
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
