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

  bool operator==(const StateMatch &other) const {
    return stateID == other.stateID && count == other.count;
  }

  bool operator!=(const StateMatch &other) const {
    return !operator==(other);
  }

  uint32_t stateID;
  unsigned count;
};

struct PatternMatch {

  /* TODO: avoid? */
  PatternMatch();

  PatternMatch(const PatternInstance &pi);

  void addStateMatch(const StateMatch &sm);

  bool hasMatch(const Word &w, unsigned &repetitions) const;

  bool hasMatch(const PatternInstance &pi, unsigned &repetitions) const;

  bool canBeMergedTo(const PatternMatch &pm,
                     std::vector<StateMatch> &result) const;

  bool operator==(const PatternMatch &other) const;

  bool operator!=(const PatternMatch &other) const;

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

bool extractPatternsForward(const ExecTree &t,
                            const std::set<uint32_t> &ids,
                            std::vector<PatternMatch> &matches);

void traverse(const ExecTree &t,
              const std::set<uint32_t> &ids,
              std::vector<TreePath> &result);

bool extractPatternsBackward(const ExecTree &t,
                             const std::set<uint32_t> &ids,
                             std::vector<PatternMatch> &matches);

}

#endif
