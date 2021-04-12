#ifndef KLEE_QUANTIFICATION_H
#define KLEE_QUANTIFICATION_H

#include "TimingSolver.h"
#include "PatternExtraction.h"
#include "Parametrization.h"

namespace klee {

ref<Expr> generateQuantifiedConstraint(PatternMatch &pm,
                                       ExecTree &tree,
                                       TimingSolver &solver);

}

#endif
