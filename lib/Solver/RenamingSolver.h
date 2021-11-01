#ifndef KLEE_RENAMINGSOLVER_H
#define KLEE_RENAMINGSOLVER_H

#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverImpl.h"

using namespace klee;

class RenamingSolver : public SolverImpl {

protected:

  Solver *solver;

public:

  RenamingSolver(Solver *solver);

  virtual ~RenamingSolver();

  bool computeTruth(const Query &query, bool &isValid);

  bool computeValidity(const Query &query, Solver::Validity &result);

  bool computeValue(const Query &query, ref<Expr> &result);

  bool computeInitialValues(const Query &query,
                            const std::vector<const Array *> &objects,
                            std::vector<std::vector<unsigned char> > &values,
                            bool &hasSolution);

  SolverRunStatus getOperationStatusCode();

  char *getConstraintLog(const Query &);

  void setCoreSolverTimeout(time::Span timeout);
};

#endif
