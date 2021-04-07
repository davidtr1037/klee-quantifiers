#ifndef KLEE_PATERN_EXTRACTION_H
#define KLEE_PATERN_EXTRACTION_H

#include "ExecTree.h"
#include "Pattern.h"

namespace klee {

struct StateMatch {
  /* TODO: add state? */
  unsigned count;
};

struct PatternMatch {
  Pattern p;
  std::vector<StateMatch> matches;
};

void extractPatterns(ExecTree &t, std::vector<PatternMatch> &matches);

}

#endif
