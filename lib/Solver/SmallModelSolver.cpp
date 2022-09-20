#include "SmallModelSolver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/Assignment.h"
#include "klee/Expr/EMatching.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"
#include "klee/Support/OptionCategories.h"
#include "klee/Support/ErrorHandling.h"
#include "klee/Statistics/TimerStatIncrementer.h"

using namespace llvm;
using namespace klee;

cl::opt<bool> ValidateModel(
  "validate-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> AddBoundConstraints(
  "add-bound-constraints",
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

cl::opt<bool> DuplicateModel(
  "duplicate-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> RepairModel(
  "repair-model",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> InstantiateAuxVariable(
  "instantiate-aux-variable",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> UseQFABVFallback(
  "use-qfabv-fallback",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

cl::opt<bool> UseLocalRepair(
  "use-local-repair",
  cl::init(false),
  cl::desc(""),
  cl::cat(SolvingCat)
);

SmallModelSolver::SmallModelSolver(Solver *solver) : solver(solver) {

}

SmallModelSolver::~SmallModelSolver() {
  klee_message("Small model hits: %lu",
               (uint64_t)(stats::smallModelHits));
  klee_message("Small model hits (strip): %lu",
               (uint64_t)(stats::smallModelStripHits));
  klee_message("Small model hits (duplicate): %lu",
               (uint64_t)(stats::smallModelDuplicateHits));
  klee_message("Small model hits (repair): %lu",
               (uint64_t)(stats::smallModelRepairHits));
  klee_message("Small model misses: %lu",
               (uint64_t)(stats::smallModelMisses));
  klee_message("Small model unsupported: %lu",
               (uint64_t)(stats::smallModelUnsupported));
  klee_message("Small model quantifier-free: %lu",
               (uint64_t)(stats::smallModelQFQueries));
  delete solver;
}

void SmallModelSolver::validate(const Query &query,
                                const std::vector<const Array *> &objects,
                                bool result) {
  std::vector<std::vector<unsigned char>> values;
  bool expected;
  if (!solver->impl->computeInitialValues(query,
                                          objects,
                                          values,
                                          expected)) {
    klee_warning("validation query failed");
    return;
  }
  assert(result == expected);
}

bool SmallModelSolver::shouldApply(const Query &query) {
  return !query.isQF();
}

bool SmallModelSolver::hasModelWithFixedAuxVars(const Query &query,
                                                const Assignment &assignment) {
  ConstraintSet constraints;
  std::vector<const Array *> objects;
  std::vector<std::vector<unsigned char>> values;

  for (auto i : assignment.bindings) {
    const Array *array = i.first;
    objects.push_back(array);
    if (array->isAuxVariable) {
      ref<Expr> aux = getSymbolicValue(array, QuantifiedExpr::AUX_VARIABLE_SIZE);
      /* TODO: consider all the bytes */
      constraints.push_back(
        EqExpr::create(
          aux,
          ConstantExpr::create(i.second[0], QuantifiedExpr::AUX_VARIABLE_WIDTH)
        )
      );
    }
  }

  for (ref<Expr> e : query.constraints) {
    constraints.push_back(e);
  }
  Query test(constraints, query.expr);

  bool hasSolution;
  assert(solver->impl->computeInitialValues(test,
                                            objects,
                                            values,
                                            hasSolution));
  return hasSolution;
}

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

bool SmallModelSolver::hasModelValue(const Assignment &assignment,
                                     const Array *object,
                                     unsigned index) {
  auto i = assignment.bindings.find(object);
  assert(i != assignment.bindings.end());
  const std::vector<unsigned char> &values = i->second;
  return index < values.size();
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
  if (index < values.size()) {
    values[index] = value;
  } else {
    klee_warning("invalid offset, skipping assignment");
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

uint64_t SmallModelSolver::getAuxValue(ref<ForallExpr> f) {
  assert(!f->auxArray);
  ref<Expr> aux = f->auxExpr;
  assert(isa<ConstantExpr>(aux));
  return dyn_cast<ConstantExpr>(aux)->getZExtValue();
}

uint64_t SmallModelSolver::getAuxValue(ref<ForallExpr> f,
                                       const Assignment &assignment) {
  if (f->auxArray) {
    ref<Expr> aux = f->auxExpr;
    ref<Expr> v = assignment.evaluate(aux);
    assert(isa<ConstantExpr>(v));
    return dyn_cast<ConstantExpr>(v)->getZExtValue();
  } else {
    return getAuxValue(f);
  }
}

static bool getMin(ref<Expr> e, ref<Expr> expected, uint64_t &min) {
  ref<UleExpr> uleExpr = dyn_cast<UleExpr>(e);
  if (uleExpr.isNull()) {
    return false;
  }

  if (*uleExpr->right != *expected) {
    return false;
  }

  ref<ConstantExpr> c = dyn_cast<ConstantExpr>(uleExpr->left);
  if (c.isNull()) {
    return false;
  }

  min = c->getZExtValue();
  return true;
}

static bool getMax(ref<Expr> e, ref<Expr> expected, uint64_t &max) {
  ref<UleExpr> uleExpr = dyn_cast<UleExpr>(e);
  if (uleExpr.isNull()) {
    return false;
  }

  if (*uleExpr->left != *expected) {
    return false;
  }

  ref<ConstantExpr> c = dyn_cast<ConstantExpr>(uleExpr->right);
  if (c.isNull()) {
    return false;
  }

  max = c->getZExtValue();
  return true;
}

void SmallModelSolver::encodeAsQF(const Query &query,
                                  ConstraintSet &constraints) {
  for (unsigned i = 0; i < query.constraints.size(); i++) {
    ref<Expr> constraint = query.constraints.get(i);
    if (isa<ForallExpr>(constraint)) {
      ref<ForallExpr> f = dyn_cast<ForallExpr>(constraint);
      if (isa<ConstantExpr>(f->auxExpr)) {
        constraints.push_back(expandForall(f));
      } else {
        assert(i >= 2);
        uint64_t min, max;
        assert(getMin(query.constraints.get(i - 2), f->auxExpr, min));
        assert(getMax(query.constraints.get(i - 1), f->auxExpr, max));
        constraints.push_back(expandForall(f, min, max));
      }
    } else {
      constraints.push_back(constraint);
    }
  }
}

ref<Expr> SmallModelSolver::eliminateForall(ref<ForallExpr> f) {
  if (!f->auxArray) {
    /* TODO: is it better to just unfold the forall? */
    uint64_t m = getAuxValue(f);
    if (m >= 1) {
      ref<Expr> e = instantiateForall(f, 1);
      if (InstantiateAuxVariable) {
        e = AndExpr::create(e, instantiateForall(f, m));
      }
      return e;
    } else {
      return ConstantExpr::create(1, Expr::Bool);
    }
  }

  ref<Expr> aux = f->auxExpr;
  ref<Expr> e = instantiateForall(f, 1);
  if (InstantiateAuxVariable) {
    e = AndExpr::create(e, instantiateForall(f, aux));
  }
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
    findUnconditionalReads(e, reads);
    for (ref<ReadExpr> r : reads) {
      if (r->index->hasAuxVariable && !r->index->hasBoundVariable) {
        /* TODO: is correct? */
        ref<Expr> index = r->index;
        ref<Expr> size = r->updates.root->sizeExpr;
        if (size.isNull()) {
          /* TODO: check when happens */
          continue;
        }

        /* TODO: always adjust to Expr::Int32? */
        if (size->getWidth() < r->index->getWidth()) {
          size = ZExtExpr::create(size, index->getWidth());
        }
        if (size->getWidth() > r->index->getWidth()) {
          index = ZExtExpr::create(index, size->getWidth());
        }

        /* avoid duplicate constraints */
        ref<Expr> constraint = UltExpr::create(index, size);
        if (std::find(result.begin(), result.end(), constraint) == result.end()) {
          result.push_back(constraint);
        }
      }
    }
  }
}

void SmallModelSolver::transform(const Query &query,
                                 ConstraintSet &constraints) {
  for (ref<Expr> e : query.constraints) {
    constraints.push_back(transform(e));
  }

  if (AddBoundConstraints) {
    std::vector<ref<Expr>> boundConstraints;
    for (ref<Expr> e : query.constraints) {
      findAuxReads(e, boundConstraints);
    }
    findAuxReads(query.expr, boundConstraints);

    for (ref<Expr> e : boundConstraints) {
      constraints.push_back(e);
    }
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

void SmallModelSolver::duplicateModel(const Query &query,
                                      Assignment &assignment,
                                      const std::vector<ArrayAccess> &conflicts) {
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

      if (!hasModelValue(assignment, access.array, access.offset)) {
        klee_warning("invalid offset, skipping duplication");
        continue;
      }

      char v = getModelValue(assignment, access.array, access.offset);
      uint64_t stopAt = InstantiateAuxVariable ? m - 1 : m;
      for (uint64_t i = 2; i <= stopAt; i++) {
        if (!getArrayAccess(body, i, access)) {
          /* shouldn't happen */
          assert(0);
        }
        /* TODO: optimize lookup */
        if (std::find(conflicts.begin(), conflicts.end(), access) == conflicts.end()) {
          setModelValue(assignment, access.array, access.offset, v);
        }
      }
    }
  }
}

void SmallModelSolver::duplicateModel(const Query &query,
                                      Assignment &assignment) {
  std::vector<ArrayAccess> conflicts;
  duplicateModel(query, assignment, conflicts);
}

/* TODO: avoid duplicate array accesses? */
void SmallModelSolver::findConflicts(const Query &query,
                                     const Assignment &assignment,
                                     bool collectAll,
                                     std::vector<ArrayAccess> &conflicts,
                                     Access2Expr &access2expr,
                                     std::set<const Array *> &nonGoundArrays) {
  std::vector<ref<Expr>> all;
  all.push_back(Expr::createIsZero(query.expr));
  for (ref<Expr> constraint : query.constraints) {
    all.push_back(constraint);
  }

  for (ref<Expr> constraint : all) {
    if (isa<ForallExpr>(constraint)) {
      ref<ForallExpr> f = dyn_cast<ForallExpr>(constraint);

      /* evaluate once */
      ref<Expr> body = assignment.evaluate(f->post);
      getArrays(body, nonGoundArrays);

      uint64_t m = getAuxValue(f, assignment);
      for (unsigned i = 1; i <= m; i++) {
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
            conflicts.push_back(access);
          }
        }
      }
    } else {
      if (collectAll) {
        std::vector<ArrayAccess> accesses;
        findDeps(constraint, assignment, accesses);
        update(access2expr, accesses, constraint);
      }
      ref<Expr> e = assignment.evaluate(constraint);
      if (e->isFalse()) {
        std::vector<ArrayAccess> accesses;
        findDeps(constraint, assignment, accesses);
        for (ArrayAccess &access : accesses) {
          conflicts.push_back(access);
        }
      }
    }
  }
}

void SmallModelSolver::collectConflictingConstraints(const std::vector<ArrayAccess> &conflicts,
                                                     const Access2Expr &access2expr,
                                                     const std::set<const Array *> &nonGoundArrays,
                                                     std::vector<ref<Expr>> &toAdd) {
  /* TODO: a better way to prevent duplicate constraints? */
  std::set<ref<Expr>> added;

  for (const ArrayAccess &dep : conflicts) {
    if (nonGoundArrays.find(dep.array) != nonGoundArrays.end()) {
      auto i = access2expr.find(dep);
      if (i != access2expr.end()) {
        const ExprBucket &exprs = i->second;
        for (ref<Expr> e : exprs) {
          if (added.find(e) == added.end()) {
            toAdd.push_back(e);
            added.insert(e);
          }
        }
      }
    }
  }
}

void SmallModelSolver::buildRepairingQuery(const Query &stripped,
                                           const std::vector<ref<Expr>> &toAdd,
                                           const Assignment &assignment,
                                           const std::set<const Array *> &nonGoundArrays,
                                           ConstraintSet &constraints,
                                           ref<Expr> &expr) {
  /* include only gound arrays */
  std::vector<const Array *> partialObjects;
  std::vector<std::vector<unsigned char>> partialValues;
  for (auto i : assignment.bindings) {
    const Array *object = i.first;
    std::vector<unsigned char> &value = i.second;
    if (nonGoundArrays.find(object) == nonGoundArrays.end()) {
      partialObjects.push_back(object);
      partialValues.push_back(value);
    }
  }

  /* we want to allow free varaibles here */
  Assignment partialAssignment(partialObjects, partialValues, true);
  for (ref<Expr> e : stripped.constraints) {
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

  expr = partialAssignment.evaluate(stripped.expr);
}

void SmallModelSolver::buildRepairingQuery(const std::vector<ref<Expr>> &toAdd,
                                           const Assignment &assignment,
                                           const std::set<const Array *> &nonGoundArrays,
                                           ConstraintSet &constraints,
                                           ref<Expr> &expr) {
  ConstraintSet empty;
  Query query(empty, ConstantExpr::create(0, Expr::Bool));
  buildRepairingQuery(query,
                      toAdd,
                      assignment,
                      nonGoundArrays,
                      constraints,
                      expr);
}

bool SmallModelSolver::repairModel(const Query &query,
                                   const Query &stripped,
                                   const Assignment &assignment,
                                   Assignment &repaired) {
  std::vector<ArrayAccess> conflicts;
  Access2Expr access2expr;
  std::set<const Array *> nonGroundArrays;
  findConflicts(query,
                assignment,
                false,
                conflicts,
                access2expr,
                nonGroundArrays);

  std::vector<ref<Expr>> toAdd;
  collectConflictingConstraints(conflicts,
                                access2expr,
                                nonGroundArrays,
                                toAdd);

  ConstraintSet constraints;
  ref<Expr> expr;
  buildRepairingQuery(stripped,
                      toAdd,
                      assignment,
                      nonGroundArrays,
                      constraints,
                      expr);

  Query partitionedQuery(constraints, expr);
  /* TODO: the order is non-deterministic */
  std::vector<const Array *> objectsToKeep(nonGroundArrays.begin(), nonGroundArrays.end());
  std::vector<std::vector<unsigned char>> valuesToKeep;
  bool hasSolution;
  bool success;
  {
    TimerStatIncrementer timer(stats::smallModelResolveQueryTime);
    success = solver->impl->computeInitialValues(partitionedQuery,
                                                 objectsToKeep,
                                                 valuesToKeep,
                                                 hasSolution);
  }
  if (!success) {
    return false;
  }

  if (!hasSolution) {
    return false;
  }

  /* construct the full assignment */
  std::vector<const Array *> fullObjects;
  std::vector<std::vector<unsigned char>> fullValues;
  for (auto i : assignment.bindings) {
    const Array *object = i.first;
    if (nonGroundArrays.find(object) == nonGroundArrays.end()) {
      std::vector<unsigned char> &value = i.second;
      fullObjects.push_back(object);
      fullValues.push_back(value);
    }
  }
  for (unsigned i = 0; i < objectsToKeep.size(); i++) {
    fullObjects.push_back(objectsToKeep[i]);
    fullValues.push_back(valuesToKeep[i]);
  }

  /* TODO: add API for this */
  for (unsigned i = 0; i < fullObjects.size(); i++) {
    repaired.bindings[fullObjects[i]] = fullValues[i];
  }

  /* TODO: eval model before patching? */
  duplicateModel(query, repaired, conflicts);
  return evalModel(query, repaired);
}

bool SmallModelSolver::repairModelLocal(const Query &query,
                                        const Assignment &assignment,
                                        Assignment &repaired) {
  std::vector<ArrayAccess> conflicts;
  Access2Expr access2expr;
  std::set<const Array *> nonGroundArrays;
  findConflicts(query,
                assignment,
                true,
                conflicts,
                access2expr,
                nonGroundArrays);

  std::vector<ref<Expr>> toAdd;
  collectConflictingConstraints(conflicts,
                                access2expr,
                                nonGroundArrays,
                                toAdd);

  /* build a query with the conflicting constraints only */
  ConstraintSet constraints;
  ref<Expr> expr;
  buildRepairingQuery(toAdd,
                      assignment,
                      nonGroundArrays,
                      constraints,
                      expr);

  /* TODO: rename? */
  Query partitionedQuery(constraints, expr);
  std::vector<const Array *> objectsToKeep(nonGroundArrays.begin(), nonGroundArrays.end());
  std::vector<std::vector<unsigned char>> valuesToKeep;
  bool hasSolution;
  bool success;
  {
    TimerStatIncrementer timer(stats::smallModelResolveQueryTime);
    success = solver->impl->computeInitialValues(partitionedQuery,
                                                 objectsToKeep,
                                                 valuesToKeep,
                                                 hasSolution);
  }
  if (!success) {
    return false;
  }

  if (!hasSolution) {
    return false;
  }

  /* construct the full assignment */
  std::vector<const Array *> fullObjects;
  std::vector<std::vector<unsigned char>> fullValues;
  for (auto i : assignment.bindings) {
    const Array *object = i.first;
    /* check if it is a ground array */
    if (nonGroundArrays.find(object) == nonGroundArrays.end()) {
      std::vector<unsigned char> &value = i.second;
      fullObjects.push_back(object);
      fullValues.push_back(value);
    }
  }
  for (unsigned i = 0; i < objectsToKeep.size(); i++) {
    const Array *array = objectsToKeep[i];
    fullObjects.push_back(array);

    auto j = assignment.bindings.find(array);
    assert(j != assignment.bindings.end());

    std::vector<unsigned char> values = j->second;
    for (const ArrayAccess &access : conflicts) {
      if (access.array == array) {
        std::vector<unsigned char> resolved = valuesToKeep[i];
        /* TODO: handle invalid accesses in an earlier stage? */
        if (access.offset < resolved.size()) {
          values[access.offset] = resolved[access.offset];
        }
      }
    }
    fullValues.push_back(values);
  }

  /* TODO: add API for this */
  for (unsigned i = 0; i < fullObjects.size(); i++) {
    repaired.bindings[fullObjects[i]] = fullValues[i];
  }

  return evalModel(query, repaired);
}

bool SmallModelSolver::adjustModel(const Query &query,
                                   const Query &stripped,
                                   const std::vector<const Array *> &objects,
                                   std::vector<std::vector<unsigned char>> &values) {
  Assignment assignment(objects, values, true);
  if (evalModel(query, assignment)) {
    ++stats::smallModelStripHits;
    return true;
  }

  if (DuplicateModel) {
    duplicateModel(query, assignment);
    if (evalModel(query, assignment)) {
      fillValues(assignment, objects, values);
      ++stats::smallModelDuplicateHits;
      return true;
    }
  }

  if (RepairModel) {
    Assignment repaired;
    bool success;
    if (UseLocalRepair) {
      success = repairModelLocal(query, assignment, repaired);
    } else {
      success = repairModel(query, stripped, assignment, repaired);
    }
    if (success) {
      fillValues(repaired, objects, values);
      ++stats::smallModelRepairHits;
      return true;
    }
  }

  return false;
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
  if (GenerateLemmasForSmallModel) {
    for (ref<Expr> e : query.constraints) {
      if (isa<ForallExpr>(e)) {
        ref<ForallExpr> f = dyn_cast<ForallExpr>(e);
        std::vector<ref<BaseAssertion>> assertions;
        findAssertions(f, assertions);

        std::vector<ref<Expr>> terms;
        for (ref<BaseAssertion> a : assertions) {
          /* guaranteed to be quantifier-free */
          a->findNegatingTerms(Expr::createIsZero(query.expr), terms);
          /* TODO: refactor */
          for (ref<Expr> constraint : query.constraints) {
            if (!isa<ForallExpr>(constraint)) {
              /* TODO: avoid trivial terms */
              a->findNegatingTerms(constraint, terms);
            }
          }
        }

        ref<Expr> aux = f->auxExpr;
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
  Query stripped(constraints, transform(query.expr));

  bool hasSmallModelSolution;
  bool success;
  {
    TimerStatIncrementer timer(stats::smallModelInitialQueryTime);
    success = solver->impl->computeInitialValues(stripped,
                                                 objects,
                                                 values,
                                                 hasSmallModelSolution);
  }
  if (!success) {
    return false;
  }

  if (hasSmallModelSolution) {
    if (!adjustModel(query, stripped, objects, values)) {
      return false;
    }
    hasSolution = true;
  } else {
    ++stats::smallModelStripHits;
    hasSolution = false;
  }

  return true;
}

bool SmallModelSolver::computeInitialValues(const Query& query,
                                            const std::vector<const Array*> &objects,
                                            std::vector<std::vector<unsigned char>> &values,
                                            bool &hasSolution) {
  TimerStatIncrementer timer(stats::smallModelTime);
  if (!shouldApply(query)) {
    ++stats::smallModelQFQueries;
    return solver->impl->computeInitialValues(query,
                                              objects,
                                              values,
                                              hasSolution);
  }

  bool success = computeInitialValuesUsingSmallModel(query,
                                                     objects,
                                                     values,
                                                     hasSolution);
  if (success) {
    if (ValidateModel) {
      validate(query, objects, hasSolution);
    }
    ++stats::smallModelHits;
    return true;
  }

  ++stats::smallModelMisses;

  /* TODO: ugly... */
  values.clear();

  if (UseQFABVFallback) {
    TimerStatIncrementer timer(stats::smallModelFallbackQueryTime);
    ConstraintSet qfConstraints;
    encodeAsQF(query, qfConstraints);
    Query qfQuery(qfConstraints, query.expr);
    success = solver->impl->computeInitialValues(qfQuery,
                                                 objects,
                                                 values,
                                                 hasSolution);
    if (ValidateModel) {
      validate(query, objects, hasSolution);
    }
  } else {
    TimerStatIncrementer timer(stats::smallModelFallbackQueryTime);
    success = solver->impl->computeInitialValues(query,
                                                 objects,
                                                 values,
                                                 hasSolution);
  }

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
