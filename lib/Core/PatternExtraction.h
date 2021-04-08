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

struct PatternMatch {

  PatternMatch(const PatternInstance &pi) {
    pattern = Pattern(pi.prefix, pi.core, pi.suffix);
    if (pi.hasCore()) {
      matches.push_back(StateMatch(pi.count));
    }
  }

  Pattern pattern;
  /* TODO: use a set? */
  std::vector<StateMatch> matches;
};

void extractPatterns(ExecTree &t, std::vector<PatternMatch> &matches);

}

#endif
