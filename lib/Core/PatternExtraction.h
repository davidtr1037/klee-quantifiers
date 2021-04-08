#ifndef KLEE_PATERN_EXTRACTION_H
#define KLEE_PATERN_EXTRACTION_H

#include "ExecTree.h"
#include "Pattern.h"

namespace klee {

/* TODO: add state? */
struct StateMatch {

  StateMatch(unsigned count) : count(count) {

  }

  unsigned count;
};

/* TODO: add constructor? */
struct PatternMatch {
  Pattern pattern;
  std::vector<StateMatch> matches;
};

void extractPatterns(ExecTree &t, std::vector<PatternMatch> &matches);

}

#endif
