#ifndef KLEE_QUANTIFICATION_H
#define KLEE_QUANTIFICATION_H

#include "PatternExtraction.h"
#include "Parametrization.h"

namespace klee {

void generateQuantifiedConstraint(const PatternMatch &pm);

}

#endif
