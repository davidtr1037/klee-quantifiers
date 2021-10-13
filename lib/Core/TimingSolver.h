//===-- TimingSolver.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_TIMINGSOLVER_H
#define KLEE_TIMINGSOLVER_H

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Solver/Solver.h"
#include "klee/System/Time.h"

#include <memory>
#include <vector>

namespace klee {
class ConstraintSet;
class Solver;

/// TimingSolver - A simple class which wraps a solver and handles
/// tracking the statistics that we care about.
class TimingSolver {
private:
  typedef std::map<const Array *, const Array *> ArrayMap;

public:
  std::unique_ptr<Solver> solver;
  bool simplifyExprs;

public:
  /// TimingSolver - Construct a new timing solver.
  ///
  /// \param _simplifyExprs - Whether expressions should be
  /// simplified (via the constraint manager interface) prior to
  /// querying.
  TimingSolver(Solver *_solver, bool _simplifyExprs = true)
      : solver(_solver), simplifyExprs(_simplifyExprs) {}

  void setTimeout(time::Span t) { solver->setCoreSolverTimeout(t); }

  char *getConstraintLog(const Query &query) {
    return solver->getConstraintLog(query);
  }

  ArrayMap rename(const Query &query,
                  ConstraintSet &constraints,
                  ref<Expr> &expr);

  void rename(const Query &query,
              const std::vector<const Array *> &objects,
              ConstraintSet &constraints,
              ref<Expr> &expr,
              std::vector<const Array *> &renamedObjects);

  bool evaluate(const ConstraintSet &, ref<Expr>, Solver::Validity &result,
                SolverQueryMetaData &metaData, bool auxiliary = false);

  bool mustBeTrue(const ConstraintSet &, ref<Expr>, bool &result,
                  SolverQueryMetaData &metaData,
                  bool auxiliary = false);

  bool mustBeFalse(const ConstraintSet &, ref<Expr>, bool &result,
                   SolverQueryMetaData &metaData,
                   bool auxiliary = false);

  bool mayBeTrue(const ConstraintSet &, ref<Expr>, bool &result,
                 SolverQueryMetaData &metaData,
                 bool auxiliary = false);

  bool mayBeFalse(const ConstraintSet &, ref<Expr>, bool &result,
                  SolverQueryMetaData &metaData,
                  bool auxiliary = false);

  bool getValue(const ConstraintSet &, ref<Expr> expr,
                ref<ConstantExpr> &result, SolverQueryMetaData &metaData);

  bool getInitialValues(const ConstraintSet &,
                        const std::vector<const Array *> &objects,
                        std::vector<std::vector<unsigned char>> &result,
                        SolverQueryMetaData &metaData,
                        bool auxiliary = false);

  std::pair<ref<Expr>, ref<Expr>> getRange(const ConstraintSet &,
                                           ref<Expr> query,
                                           SolverQueryMetaData &metaData);
};
}

#endif /* KLEE_TIMINGSOLVER_H */
