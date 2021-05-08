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

  bool hasMatch(const Word &w, unsigned &repetitions) const;

  bool hasMatch(const PatternInstance &pi, unsigned &repetitions) const;

  bool canBeMergedTo(const PatternMatch &pm,
                     std::vector<StateMatch> &result) const;

  void dump() const;

  Pattern pattern;
  /* TODO: use a set? */
  std::vector<StateMatch> matches;
};

struct TreePath {
  Word w;
  uint32_t stateID;

  TreePath() : stateID(0) {

  }

  TreePath(Word &w, uint32_t stateID) : w(w), stateID(stateID) {

  }
};

void extractPatterns(ExecTree &t,
                     std::set<uint32_t> &ids,
                     std::vector<PatternMatch> &matches);

void traverse(ExecTree &t,
              std::set<uint32_t> &ids,
              std::vector<TreePath> &result);

void extractPatternsBackward(ExecTree &t,
                             std::set<uint32_t> &ids,
                             std::vector<PatternMatch> &matches);

}

#endif
