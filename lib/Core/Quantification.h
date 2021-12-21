#ifndef KLEE_QUANTIFICATION_H
#define KLEE_QUANTIFICATION_H

#include "TimingSolver.h"
#include "PatternExtraction.h"
#include "Parametrization.h"

namespace klee {

ref<Expr> generateQuantifiedConstraint(PatternMatch &pm,
                                       ExecTree &tree,
                                       uint32_t mergeID,
                                       TimingSolver &solver);

bool generateMergedValue(PatternMatch &pm,
                         ExecTree &tree,
                         State2Value &valuesMap,
                         uint32_t mergeID,
                         TimingSolver &solver,
                         ParametrizedExpr &solution);

}

#endif
