#include "SmallModelSolver.h"
#include "klee/Solver/Solver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/Assignment.h"
#include "klee/Expr/EMatching.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"
#include "klee/Support/OptionCategories.h"
#include "klee/Support/ErrorHandling.h"

using namespace llvm;
using namespace klee;

cl::opt<bool> ValidateSmallModel(
  "validate-small-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> GenerateLemmasForSmallModel(
  "generate-lemmas-for-small-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> AdjustForConflicts(
  "adjust-for-conflicts",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

static void update(Access2Expr &map,
                   std::vector<ArrayAccess> &accesses,
                   ref<Expr> e) {
  for (ArrayAccess &access : accesses) {
    ExprBucket &exprs = map[access];
    /* TODO: use a set while preserving insertion order? */
    if (std::find(exprs.begin(), exprs.end(), e) == exprs.end()) {
      exprs.push_back(e);
    }
  }
}

static void fillValues(const Assignment &assignment,
                       const std::vector<const Array *> &objects,
                       std::vector<std::vector<unsigned char>> &values) {
  values.clear();
  for (const Array *object : objects) {
    auto i = assignment.bindings.find(object);
    assert(i != assignment.bindings.end());
    values.push_back(i->second);
  }
}

void SmallModelSolver::dumpModel(const Assignment &assignment) {
  for (auto i : assignment.bindings) {
    const Array *array = i.first;
    std::vector<unsigned char> bytes = i.second;
    errs() << array->getName() << "\n";
    for (unsigned j = 0; j < bytes.size(); j++) {
      errs() << (unsigned)(bytes[j]) << " ";
    }
    errs() << "\n";
  }
}

char SmallModelSolver::getModelValue(const Assignment &assignment,
                                     const Array *object,
                                     unsigned index) {
  auto i = assignment.bindings.find(object);
  assert(i != assignment.bindings.end());
  const std::vector<unsigned char> &values = i->second;
  assert(index < values.size());
  return values[index];
}

void SmallModelSolver::setModelValue(Assignment &assignment,
                                     const Array *object,
                                     unsigned index,
                                     char value) {
  auto i = assignment.bindings.find(object);
  assert(i != assignment.bindings.end());
  std::vector<unsigned char> &values = i->second;
  assert(index < values.size());
  values[index] = value;
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
  if (!f->auxArray) {
    uint64_t m = getAuxValue(f);
    if (m >= 1) {
      return instantiateForall(f, 1);
    } else {
      return ConstantExpr::create(1, Expr::Bool);
    }
  }

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

uint64_t SmallModelSolver::getAuxValue(ref<ForallExpr> f) {
  assert(!f->auxArray);
  assert(isa<AndExpr>(f->pre));
  ref<AndExpr> andExpr = dyn_cast<AndExpr>(f->pre);
  assert(isa<UleExpr>(andExpr->right));
  ref<UleExpr> uleExpr = dyn_cast<UleExpr>(andExpr->right);
  assert(isa<ConstantExpr>(uleExpr->right));
  return dyn_cast<ConstantExpr>(uleExpr->right)->getZExtValue();
}

uint64_t SmallModelSolver::getAuxValue(ref<ForallExpr> f,
                                       const Assignment &assignment) {
  if (f->auxArray) {
    ref<Expr> aux = getSymbolicValue(f->auxArray,
                                     QuantifiedExpr::AUX_VARIABLE_SIZE);
    ref<Expr> v = assignment.evaluate(aux);
    assert(isa<ConstantExpr>(v));
    return dyn_cast<ConstantExpr>(v)->getZExtValue();
  } else {
    return getAuxValue(f);
  }
}

void SmallModelSolver::findDeps(ref<Expr> e,
                                const Assignment &assignment,
                                std::vector<ArrayAccess> &result) {
  if (!isa<ForallExpr>(e)) {
    std::vector<ref<ReadExpr>> reads;
    findReads(e, true, reads);
    for (ref<ReadExpr> r : reads) {
      if (r->updates.root->isSymbolicArray()) {
        ref<Expr> index = assignment.evaluate(r->index);
        assert(isa<ConstantExpr>(index));
        uint64_t offset = dyn_cast<ConstantExpr>(index)->getZExtValue();
        result.push_back(ArrayAccess(r->updates.root, offset));
      }
    }
  } else {
    ref<ForallExpr> f = dyn_cast<ForallExpr>(e);
    uint64_t m = getAuxValue(f, assignment);
    for (unsigned i = 1; i <= m; i++) {
      ref<Expr> e = instantiateForall(f, i);
      findDeps(e, assignment, result);
    }
  }
}

void SmallModelSolver::findAuxReads(ref<Expr> e,
                                    std::vector<ref<Expr>> &result) {
  if (e->hasAuxVariable) {
    std::vector<ref<ReadExpr>> reads;
    findReads(e, true, reads);
    for (ref<ReadExpr> r : reads) {
      if (r->index->hasAuxVariable && !r->index->hasBoundVariable) {
        /* TODO: is correct? */
        ref<Expr> index = r->index;
        ref<Expr> size = r->updates.root->sizeExpr;
        if (size.isNull()) {
          /* TODO: check when happens */
          continue;
        }

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

/* TODO: rename */
void SmallModelSolver::getArrays(ref<Expr> e,
                                 std::set<const Array *> &result) {
  std::vector<ref<ReadExpr>> reads;
  findReads(e, true, reads);
  for (ref<ReadExpr> r : reads) {
    if (r->index->hasBoundVariable && r->updates.root->isSymbolicArray()) {
      result.insert(r->updates.root);
    }
  }
}

void SmallModelSolver::getArrayAccesses(ref<Expr> e,
                                        uint64_t value,
                                        std::vector<ArrayAccess> &result) {
  std::vector<ref<ReadExpr>> reads;
  findReads(e, true, reads);
  for (ref<ReadExpr> r : reads) {
    if (r->index->hasBoundVariable && r->updates.root->isSymbolicArray()) {
      ref<Expr> index = substBoundVariables(r->index, value);
      if (isa<ConstantExpr>(index)) {
        uint64_t offset = dyn_cast<ConstantExpr>(index)->getZExtValue();
        const Array *array = r->updates.root;
        result.push_back(ArrayAccess(array, offset));
      }
    }
  }
}

bool SmallModelSolver::getArrayAccess(ref<Expr> e,
                                      uint64_t value,
                                      ArrayAccess &result) {
  std::vector<ArrayAccess> accesses;
  getArrayAccesses(e, value, accesses);
  if (!accesses.empty()) {
    result = accesses[0];
    return true;
  }

  return false;
}

void SmallModelSolver::extendModel(const Query &query,
                                   Assignment &assignment,
                                   std::vector<ArrayAccess> conflicting) {
  for (ref<Expr> e : query.constraints) {
    if (isa<ForallExpr>(e)) {
      ref<ForallExpr> f = dyn_cast<ForallExpr>(e);
      uint64_t m = getAuxValue(f, assignment);
      if (m == 0) {
        /* the forall condition is trivially satisfied */
        continue;
      }

      ref<Expr> body = assignment.evaluate(f->post);
      ArrayAccess access;
      if (!getArrayAccess(body, 1, access)) {
        /* there are no reads that depend on the bound variable */
        continue;
      }

      char c = getModelValue(assignment, access.array, access.offset);
      for (uint64_t v = 2; v <= m; v++) {
        if (!getArrayAccess(body, v, access)) {
          /* shouldn't happen */
          assert(0);
        }
        /* TODO: optimize lookup */
        if (std::find(conflicting.begin(), conflicting.end(), access) == conflicting.end()) {
          setModelValue(assignment, access.array, access.offset, c);
        }
      }
    }
  }
}

void SmallModelSolver::extendModel(const Query &query,
                                   Assignment &assignment) {
  std::vector<ArrayAccess> conflicting;
  extendModel(query, assignment, conflicting);
}

bool SmallModelSolver::adjustModelForConflicts(const Query &query,
                                               const Query &smQuery,
                                               const Assignment &assignment,
                                               Assignment &adjusted) {
  std::vector<ref<Expr>> all;
  all.push_back(Expr::createIsZero(query.expr));
  for (ref<Expr> constraint : query.constraints) {
    all.push_back(constraint);
  }

  std::set<const Array *> keepSymbolic;
  /* TODO: rename to conflicts? */
  std::vector<ArrayAccess> conflicting;
  Access2Expr access2expr;
  /* TODO: map an access to a set of instantiations */
  for (ref<Expr> constraint : all) {
    if (isa<ForallExpr>(constraint)) {
      ref<ForallExpr> f = dyn_cast<ForallExpr>(constraint);

      /* evaluate once */
      ref<Expr> body = assignment.evaluate(f->post);
      getArrays(body, keepSymbolic);

      uint64_t m = getAuxValue(f, assignment);
      for (unsigned i = 1; i <= m; i++) {
        /* TODO: use substBoundVariables? */
        ref<Expr> instantiation = instantiateForall(f, i);
        ref<Expr> e = assignment.evaluate(instantiation);
        std::vector<ArrayAccess> accesses;
        getArrayAccesses(body, i, accesses);
        if (i != 1) {
          /* the first instantiation will be added anyway */
          update(access2expr, accesses, instantiation);
        }
        if (e->isFalse()) {
          for (ArrayAccess &access : accesses) {
            conflicting.push_back(access);
          }
        }
      }
    } else {
      ref<Expr> e = assignment.evaluate(constraint);
      std::vector<ArrayAccess> accesses;
      findDeps(constraint, assignment, accesses);
      if (e->isFalse()) {
        for (ArrayAccess &access : accesses) {
          conflicting.push_back(access);
        }
      }
    }
  }

  /* TODO: for each conflict, add the associated constraints */
  std::vector<ref<Expr>> toAdd;
  for (ArrayAccess &dep : conflicting) {
    if (keepSymbolic.find(dep.array) != keepSymbolic.end()) {
      auto i = access2expr.find(dep);
      if (i != access2expr.end()) {
        ExprBucket &exprs = i->second;
        for (ref<Expr> e : exprs) {
          toAdd.push_back(e);
        }
      }
    }
  }

  std::vector<const Array *> partialObjects;
  std::vector<std::vector<unsigned char>> partialValues;
  for (auto i : assignment.bindings) {
    const Array *object = i.first;
    std::vector<unsigned char> &value = i.second;
    if (keepSymbolic.find(object) == keepSymbolic.end()) {
      partialObjects.push_back(object);
      partialValues.push_back(value);
    }
  }

  Assignment partialAssignment(partialObjects, partialValues, true);
  ConstraintSet constraints;
  for (ref<Expr> e : smQuery.constraints) {
    ref<Expr> substituted = partialAssignment.evaluate(e);
    /* TODO: what if evaluated to false? */
    if (!isa<ConstantExpr>(substituted)) {
      constraints.push_back(substituted);
    }
  }
  for (ref<Expr> e : toAdd) {
    ref<Expr> substituted = partialAssignment.evaluate(e);
    if (!isa<ConstantExpr>(substituted)) {
      constraints.push_back(substituted);
    }
  }

  Query partitionedQuery(constraints, partialAssignment.evaluate(smQuery.expr));
  std::vector<const Array *> objectsToKeep(keepSymbolic.begin(), keepSymbolic.end());
  std::vector<std::vector<unsigned char>> valuesToKeep;
  bool hasSolution;
  bool success = solver->impl->computeInitialValues(partitionedQuery,
                                                    objectsToKeep,
                                                    valuesToKeep,
                                                    hasSolution);
  if (!success) {
    return false;
  }

  if (!hasSolution) {
    return false;
  }

  /* construct the full assignment */
  std::vector<const Array *> fullObjects = partialObjects;
  std::vector<std::vector<unsigned char>> fullValues = partialValues;
  for (unsigned i = 0; i < objectsToKeep.size(); i++) {
    fullObjects.push_back(objectsToKeep[i]);
    fullValues.push_back(valuesToKeep[i]);
  }

  for (unsigned i = 0; i < fullObjects.size(); i++) {
    adjusted.bindings[fullObjects[i]] = fullValues[i];
  }

  /* TODO: eval model? */

  /* TODO: extend (with conflicts) and eval model */
  extendModel(query, adjusted, conflicting);
  if (evalModel(query, adjusted)) {
    return true;
  }

  return false;
}

bool SmallModelSolver::adjustModel(const Query &query,
                                   const Query &smQuery,
                                   const std::vector<const Array *> &objects,
                                   std::vector<std::vector<unsigned char>> &values) {
  Assignment assignment(objects, values, true);
  if (evalModel(query, assignment)) {
    return true;
  }

  extendModel(query, assignment);
  if (evalModel(query, assignment)) {
    fillValues(assignment, objects, values);
    return true;
  }

  if (AdjustForConflicts) {
    Assignment adjusted;
    if (adjustModelForConflicts(query, smQuery, assignment, adjusted)) {
      fillValues(adjusted, objects, values);
      return true;
    }
  }

  return false;
}

SmallModelSolver::SmallModelSolver(Solver *solver) : solver(solver) {

}

SmallModelSolver::~SmallModelSolver() {
  klee_message("Small model hits: %lu",
               (uint64_t)(stats::smallModelHits));
  klee_message("Small model misses: %lu",
               (uint64_t)(stats::smallModelMisses));
  klee_message("Small model unsupported: %lu",
               (uint64_t)(stats::smallModelUnsupported));
  delete solver;
}

bool SmallModelSolver::computeTruth(const Query& query,
                                    bool &isValid) {
  ++stats::smallModelMisses;
  return solver->impl->computeTruth(query, isValid);
}

bool SmallModelSolver::computeValidity(const Query& query,
                                       Solver::Validity &result) {
  ++stats::smallModelMisses;
  return solver->impl->computeValidity(query, result);
}

bool SmallModelSolver::computeValue(const Query& query,
                                    ref<Expr> &result) {
  ++stats::smallModelMisses;
  return solver->impl->computeValue(query, result);
}

void SmallModelSolver::buildConstraints(const Query &query,
                                        ConstraintSet &constraints) {
  if (GenerateLemmasForSmallModel && !isa<ConstantExpr>(query.expr)) {
    for (ref<Expr> e : query.constraints) {
      if (isa<ForallExpr>(e)) {
        std::vector<EqAssertion> assertions;
        ref<ForallExpr> f = dyn_cast<ForallExpr>(e);
        findAssertions(f, assertions);

        std::vector<ref<Expr>> terms;
        for (EqAssertion &a : assertions) {
          a.findNegatingTerms(Expr::createIsZero(query.expr), terms);
        }

        ref<Expr> aux = getSymbolicValue(f->auxArray, f->auxArray->size);
        for (ref<Expr> term : terms) {
          term = ZExtExpr::create(term, QuantifiedExpr::AUX_VARIABLE_WIDTH);
          ref<Expr> lemma = OrExpr::create(
            EqExpr::create(ConstantExpr::create(0, term->getWidth()), aux),
            OrExpr::create(
              UltExpr::create(term, ConstantExpr::create(1, term->getWidth())),
              UltExpr::create(aux, term)
            )
          );
          constraints.push_back(lemma);
        }
      }
    }
  }

  /* TODO: rename */
  transform(query, constraints);
}

bool SmallModelSolver::computeInitialValuesUsingSmallModel(const Query &query,
                                                           const std::vector<const Array *> &objects,
                                                           std::vector<std::vector<unsigned char>> &values,
                                                           bool &hasSolution) {
  ConstraintSet constraints;
  buildConstraints(query, constraints);
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
    if (!adjustModel(query, smQuery, objects, values)) {
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
    ++stats::smallModelHits;
    return true;
  }

  ++stats::smallModelMisses;

  /* TODO: ugly... */
  values.clear();
  success = solver->impl->computeInitialValues(query,
                                               objects,
                                               values,
                                               hasSolution);
  if (!hasSolution) {
    ++stats::smallModelUnsupported;
  }
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
