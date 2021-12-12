#ifndef KLEE_SMALLMODELSOLVER_H
#define KLEE_SMALLMODELSOLVER_H

#include "klee/Expr/Assignment.h"
#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverImpl.h"

using namespace klee;

class SmallModelSolver : public SolverImpl {

protected:

  Solver *solver;

public:

  SmallModelSolver(Solver *solver);

  virtual ~SmallModelSolver();

  void dumpModel(const std::vector<const Array *> &objects,
                 const std::vector<std::vector<unsigned char>> &values);

  bool evalModel(const Query &query,
                 Assignment &assignment);

  ref<Expr> eliminateForall(ref<ForallExpr>);

  ref<Expr> transform(ref<Expr> e);

  void findAuxReads(ref<Expr> e,
                    std::vector<ref<Expr>> &result);

  void transform(const Query &query,
                 ConstraintSet &constraints);

  bool getAccessedOffset(ref<Expr> e,
                         uint64_t value,
                         uint64_t &offset,
                         const Array *&array);

  char getModelValue(const std::vector<const Array *> &objects,
                     std::vector<std::vector<unsigned char>> &values,
                     const Array *object,
                     unsigned index);

  void setModelValue(const std::vector<const Array *> &objects,
                     std::vector<std::vector<unsigned char>> &values,
                     const Array *object,
                     unsigned index,
                     char value);

  bool adjustModel(const Query &query,
                   const std::vector<const Array *> &objects,
                   std::vector<std::vector<unsigned char>> &values);

  bool computeTruthUsingSmallModel(const Query &query, bool &isValid);

  bool computeTruth(const Query &query, bool &isValid);

  bool computeValidity(const Query &query, Solver::Validity &result);

  bool computeValue(const Query &query, ref<Expr> &result);

  bool computeInitialValuesUsingSmallModel(const Query &query,
                                           const std::vector<const Array *> &objects,
                                           std::vector<std::vector<unsigned char> > &values,
                                           bool &hasSolution);

  bool computeInitialValues(const Query &query,
                            const std::vector<const Array *> &objects,
                            std::vector<std::vector<unsigned char> > &values,
                            bool &hasSolution);

  SolverRunStatus getOperationStatusCode();

  char *getConstraintLog(const Query &);

  void setCoreSolverTimeout(time::Span timeout);
};

#endif
