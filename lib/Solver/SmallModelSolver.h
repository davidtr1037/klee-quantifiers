#ifndef KLEE_SMALLMODELSOLVER_H
#define KLEE_SMALLMODELSOLVER_H

#include "klee/Expr/Assignment.h"
#include "klee/Solver/Solver.h"
#include "klee/Solver/SolverImpl.h"

using namespace klee;

struct ArrayAccess {
  const Array *array;
  uint64_t offset;

  ArrayAccess() :
    array(nullptr), offset(0) {

  }

  ArrayAccess(const Array *array, uint64_t offset) :
    array(array), offset(offset) {

  }

  bool operator==(const ArrayAccess &other) const {
    return array == other.array && offset == other.offset;
  }
};

struct ArrayAccessHash {

  std::size_t operator() (const ArrayAccess &access) const {
    std::size_t h1 = std::hash<const Array *>()(access.array);
    std::size_t h2 = std::hash<uint64_t>()(access.offset);
    return h1 ^ h2;
  }

};

typedef std::vector<ref<Expr>> ExprBucket;
typedef std::unordered_map<ArrayAccess, ExprBucket, ArrayAccessHash> Access2Expr;

class SmallModelSolver : public SolverImpl {

protected:

  Solver *solver;

public:

  SmallModelSolver(Solver *solver);

  virtual ~SmallModelSolver();

  void dumpModel(const Assignment &assignment);

  char getModelValue(const Assignment &assignment,
                     const Array *object,
                     unsigned index);

  void setModelValue(Assignment &assignment,
                     const Array *object,
                     unsigned index,
                     char value);

  bool evalModel(const Query &query,
                 Assignment &assignment);

  ref<Expr> eliminateForall(ref<ForallExpr>);

  ref<Expr> transform(ref<Expr> e);

  uint64_t getAuxValue(ref<ForallExpr> f);

  uint64_t getAuxValue(ref<ForallExpr> f,
                       const Assignment &assignment);

  void findDeps(ref<Expr> e,
                const Assignment &assignment,
                std::vector<ArrayAccess> &result);

  void findAuxReads(ref<Expr> e,
                    std::vector<ref<Expr>> &result);

  void transform(const Query &query,
                 ConstraintSet &constraints);

  void getArrays(ref<Expr> e,
                 std::set<const Array *> &result);

  void getArrayAccesses(ref<Expr> e,
                        uint64_t value,
                        std::vector<ArrayAccess> &result);

  bool getArrayAccess(ref<Expr> e,
                      uint64_t value,
                      ArrayAccess &result);

  void extendModel(const Query &query,
                   Assignment &assignment,
                   std::vector<ArrayAccess> conflicting);

  void extendModel(const Query &query,
                   Assignment &assignment);

  /* TODO: the third parameter should be const */
  bool adjustModelForConflicts(const Query &query,
                               const Query &smQuery,
                               const Assignment &assignment,
                               Assignment &adjusted);

  bool adjustModel(const Query &query,
                   const Query &smQuery,
                   const std::vector<const Array *> &objects,
                   std::vector<std::vector<unsigned char>> &values);

  bool computeTruthUsingSmallModel(const Query &query, bool &isValid);

  bool computeTruth(const Query &query, bool &isValid);

  bool computeValidity(const Query &query, Solver::Validity &result);

  bool computeValue(const Query &query, ref<Expr> &result);

  void buildConstraints(const Query &query,
                        ConstraintSet &constraints);

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
