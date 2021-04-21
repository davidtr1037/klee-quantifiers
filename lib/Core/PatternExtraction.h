#ifndef KLEE_PATERN_EXTRACTION_H
#define KLEE_PATERN_EXTRACTION_H

#include "ExecTree.h"
#include "Pattern.h"

#include <vector>
#include <set>

namespace klee {

struct StateMatch {

  StateMatch(uint32_t stateID, unsigned count) : stateID(stateID), count(count) {

  }

  uint32_t stateID;
  unsigned count;
};

struct PatternMatch {

  PatternMatch(const PatternInstance &pi);

  void addStateMatch(const StateMatch &sm);

  Pattern pattern;
  /* TODO: use a set? */
  std::vector<StateMatch> matches;
};

void extractPatterns(ExecTree &t,
                     std::set<uint32_t> &ids,
                     std::vector<PatternMatch> &matches);

}

#endif
