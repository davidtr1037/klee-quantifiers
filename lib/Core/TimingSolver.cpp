//===-- TimingSolver.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "TimingSolver.h"

#include "ExecutionState.h"

#include "klee/Config/Version.h"
#include "klee/Statistics/Statistics.h"
#include "klee/Statistics/TimerStatIncrementer.h"
#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverStats.h"
#include "klee/Expr/ExprRename.h"

#include "CoreStats.h"

using namespace klee;
using namespace llvm;

/***/

cl::opt<bool> RenameExpr("rename-expr", cl::init(false), cl::desc(""));

bool TimingSolver::evaluate(const ConstraintSet &constraints, ref<Expr> expr,
                            Solver::Validity &result,
                            SolverQueryMetaData &metaData,
                            bool auxiliary) {
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? Solver::True : Solver::False;
    return true;
  }

  TimerStatIncrementer timer(stats::solverTime);

  if (auxiliary) {
    ++stats::auxilaryQueries;
  }

  if (simplifyExprs)
    expr = ConstraintManager::simplifyExpr(constraints, expr);

  bool success;
  if (RenameExpr) {
    ConstraintSet renamedConstraints;
    ref<Expr> renamedExpr;
    rename(Query(constraints, expr), renamedConstraints, renamedExpr);
    Query renamed = Query(renamedConstraints, renamedExpr);
    success = solver->evaluate(renamed, result);
  } else {
    success = solver->evaluate(Query(constraints, expr), result);
  }

  metaData.queryCost += timer.delta();

  return success;
}

bool TimingSolver::mustBeTrue(const ConstraintSet &constraints,
                              ref<Expr> expr,
                              bool &result,
                              SolverQueryMetaData &metaData,
                              bool auxiliary) {
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE->isTrue() ? true : false;
    return true;
  }

  TimerStatIncrementer timer(stats::solverTime);

  if (auxiliary) {
    ++stats::auxilaryQueries;
  }

  if (simplifyExprs)
    expr = ConstraintManager::simplifyExpr(constraints, expr);

  bool success;
  if (RenameExpr) {
    ConstraintSet renamedConstraints;
    ref<Expr> renamedExpr;
    rename(Query(constraints, expr), renamedConstraints, renamedExpr);
    Query renamed = Query(renamedConstraints, renamedExpr);
    success = solver->mustBeTrue(renamed, result);
  } else {
    success = solver->mustBeTrue(Query(constraints, expr), result);
  }

  metaData.queryCost += timer.delta();

  return success;
}

bool TimingSolver::mustBeFalse(const ConstraintSet &constraints,
                               ref<Expr> expr,
                               bool &result,
                               SolverQueryMetaData &metaData,
                               bool auxiliary) {
  return mustBeTrue(constraints,
                    Expr::createIsZero(expr),
                    result,
                    metaData,
                    auxiliary);
}

bool TimingSolver::mayBeTrue(const ConstraintSet &constraints,
                             ref<Expr> expr,
                             bool &result,
                             SolverQueryMetaData &metaData,
                             bool auxiliary) {
  bool res;
  if (!mustBeFalse(constraints, expr, res, metaData, auxiliary))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::mayBeFalse(const ConstraintSet &constraints,
                              ref<Expr> expr,
                              bool &result,
                              SolverQueryMetaData &metaData,
                              bool auxiliary) {
  bool res;
  if (!mustBeTrue(constraints, expr, res, metaData, auxiliary))
    return false;
  result = !res;
  return true;
}

bool TimingSolver::getValue(const ConstraintSet &constraints,
                            ref<Expr> expr,
                            ref<ConstantExpr> &result,
                            SolverQueryMetaData &metaData) {
  // Fast path, to avoid timer and OS overhead.
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(expr)) {
    result = CE;
    return true;
  }
  
  TimerStatIncrementer timer(stats::solverTime);

  if (simplifyExprs)
    expr = ConstraintManager::simplifyExpr(constraints, expr);

  bool success;
  if (RenameExpr) {
    ConstraintSet renamedConstraints;
    ref<Expr> renamedExpr;
    rename(Query(constraints, expr), renamedConstraints, renamedExpr);
    Query renamed = Query(renamedConstraints, renamedExpr);
    success = solver->getValue(renamed, result);
  } else {
    success = solver->getValue(Query(constraints, expr), result);
  }

  metaData.queryCost += timer.delta();

  return success;
}

bool TimingSolver::getInitialValues(
    const ConstraintSet &constraints,
    const std::vector<const Array *> &objects,
    std::vector<std::vector<unsigned char>> &result,
    SolverQueryMetaData &metaData,
    bool auxiliary) {
  if (objects.empty())
    return true;

  TimerStatIncrementer timer(stats::solverTime);

  if (auxiliary) {
    ++stats::auxilaryQueries;
  }

  bool success;
  if (RenameExpr) {
    ConstraintSet renamedConstraints;
    ref<Expr> renamedExpr;
    std::vector<const Array *> renamedObjects;
    rename(
      Query(constraints, ConstantExpr::alloc(0, Expr::Bool)),
      objects,
      renamedConstraints,
      renamedExpr,
      renamedObjects
    );
    success = solver->getInitialValues(
      Query(renamedConstraints, renamedExpr), renamedObjects, result);
  } else {
    success = solver->getInitialValues(
      Query(constraints, ConstantExpr::alloc(0, Expr::Bool)), objects, result);
  }

  metaData.queryCost += timer.delta();

  return success;
}

std::pair<ref<Expr>, ref<Expr>>
TimingSolver::getRange(const ConstraintSet &constraints, ref<Expr> expr,
                       SolverQueryMetaData &metaData) {
  TimerStatIncrementer timer(stats::solverTime);
  auto result = solver->getRange(Query(constraints, expr));
  metaData.queryCost += timer.delta();
  return result;
}
