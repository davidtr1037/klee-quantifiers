#include "PatternExtraction.h"

#include <algorithm>

using namespace llvm;
using namespace klee;

static bool compare(StateMatch &sm1, StateMatch &sm2) {
  return sm1.count < sm2.count;
}

PatternMatch::PatternMatch() {

}

PatternMatch::PatternMatch(const PatternInstance &pi) {
  pattern = Pattern(pi.prefix, pi.core, pi.suffix);
}

/* TODO: sort by sm.count? */
void PatternMatch::addStateMatch(const StateMatch &sm) {
  if (!pattern.hasCore()) {
    /* if has no core, only one match is possible */
    assert(matches.empty());
  }

  /* TODO: check inconsistencies? */
  matches.push_back(sm);
}

bool PatternMatch::hasMatch(const Word &w,
                            unsigned &repetitions) const {
  if (!pattern.hasCore()) {
    Word instance = pattern.getInstance(0);
    if (instance == w) {
      repetitions = 0;
      return true;
    } else {
      return false;
    }
  }

  /* TODO: don't check existing state matches */
  unsigned k = 0;
  Word instance;
  do {
    instance = pattern.getInstance(k);
    if (instance == w) {
      repetitions = k;
      return true;
    }
    k++;
  } while (instance.size() < w.size());

  return false;
}

bool PatternMatch::hasMatch(const PatternInstance &pi,
                            unsigned &repetitions) const {
  if (!pattern.hasCore()) {
    /* TODO: why? */
    return false;
  }

  /* TODO: comparing different types here... */
  if (pattern == pi) {
    /* the number of repetitions is already given */
    repetitions = pi.count;
    return true;
  }

  return hasMatch(pi.word, repetitions);
}

bool PatternMatch::canBeMergedTo(const PatternMatch &pm,
                                 std::vector<StateMatch> &result) const {
  for (const StateMatch &sm : matches) {
    Word w = pattern.getInstance(sm.count);
    unsigned repetitions;
    if (!pm.hasMatch(w, repetitions)) {
      return false;
    }
    result.push_back(StateMatch(sm.stateID, repetitions));
  }
  return true;
}

bool PatternMatch::operator==(const PatternMatch &other) const {
  return pattern == other.pattern && matches == other.matches;
}

bool PatternMatch::operator!=(const PatternMatch &other) const {
  return !operator==(other);
}

void PatternMatch::dump() const {
  errs() << "pattern:\n";
  pattern.dump();
  for (auto &sm : matches) {
    errs() << "-- count " << sm.count << "\n";
  }
}

static bool addPattern(std::vector<PatternMatch> &matches,
                       PatternInstance &pi,
                       uint32_t stateID) {
  for (PatternMatch &pm : matches) {
    unsigned count;
    if (pm.hasMatch(pi, count)) {
      pm.matches.push_back(StateMatch(stateID, count));
      return true;
    }
  }

  PatternMatch pm(pi);
  pm.addStateMatch(StateMatch(stateID, pi.count));
  matches.push_back(pm);
  return false;
}

static void handleLeaf(ExecTreeNode *n,
                       PatternInstance &pi,
                       std::vector<PatternMatch> &matches) {
  addPattern(matches, pi, n->stateID);
}

/* TODO: add const */
static bool hasCommonBoundaries(const PatternMatch &pm1,
                                const PatternMatch &pm2,
                                PatternMatch &unified) {
  assert(!pm1.pattern.hasCore() && !pm2.pattern.hasCore());

  Word w1, w2;
  if (pm1.pattern.prefix.size() > pm2.pattern.prefix.size()) {
    w1 = pm2.pattern.prefix;
    w2 = pm1.pattern.prefix;
  } else {
    w1 = pm1.pattern.prefix;
    w2 = pm2.pattern.prefix;
  }

  Word prefix, core, suffix;
  prefix = Word::getCommonPrefix(w1, w2);
  suffix = Word::getCommonSuffix(w1, w2);

  unsigned boundarySize = prefix.size() + suffix.size();
  if (boundarySize > 0 && boundarySize < w2.size()) {
    for (unsigned i = prefix.size(); i < w2.size() - suffix.size(); i++) {
      core.append(w2[i]);
    }
    unified.pattern = Pattern(prefix, core, suffix);

    uint32_t id1 = pm1.matches[0].stateID;
    uint32_t id2 = pm2.matches[0].stateID;
    unified.addStateMatch(StateMatch(id1, pm1.pattern.prefix == w1 ? 0 : 1));
    unified.addStateMatch(StateMatch(id2, pm1.pattern.prefix == w1 ? 1 : 0));
    return true;
  }

  return false;
}

static bool unifyMatches(std::vector<std::pair<PatternMatch, bool>> &worklist) {
  for (auto &i1 : worklist) {
    PatternMatch &pm1 = i1.first;
    for (auto &i2 : worklist) {
      if (i1.second || i2.second) {
        /* already merged */
        continue;
      }

      PatternMatch &pm2 = i2.first;
      if (pm1 == pm2) {
        continue;
      }

      if (pm1.matches.size() <= pm2.matches.size()) {
        std::vector<StateMatch> mergable;
        if (pm1.canBeMergedTo(pm2, mergable)) {
          for (StateMatch &sm : mergable) {
            pm2.addStateMatch(sm);
          }
          i1.second = true;
          break;
        }
      }

      PatternMatch unified;
      if (!pm1.pattern.hasCore() && !pm2.pattern.hasCore()) {
        if (hasCommonBoundaries(pm1, pm2, unified)) {
          i1.second = i2.second = true;
          worklist.push_back(std::make_pair(unified, false));
          return true;
        }
      }
    }
  }

  return false;
}

/* TODO: do we need a fixpoint algorithm here? */
static void unifyMatches(std::vector<PatternMatch> &matches,
                         std::vector<PatternMatch> &result) {
  /* we can first sort, and then do the unification */
  std::vector<std::pair<PatternMatch, bool>> worklist;
  for (PatternMatch &pm : matches) {
    worklist.push_back(std::make_pair(pm, false));
  }

  bool changed;
  do {
    changed = unifyMatches(worklist);
  } while (changed);

  for (auto &i : worklist) {
    /* add if not merged */
    if (!i.second) {
      result.push_back(i.first);
    }
  }
}

void klee::extractPatterns(ExecTree &t,
                           std::set<uint32_t> &ids,
                           std::vector<PatternMatch> &result) {
  std::vector<PatternMatch> matches;
  std::vector<std::pair<ExecTreeNode *, PatternInstance>> worklist;

  /* initialize the worklist */
  worklist.push_back(std::make_pair(t.root, PatternInstance()));

  while (!worklist.empty()) {
    auto p = worklist.back();
    worklist.pop_back();

    ExecTreeNode *n = p.first;
    PatternInstance pi = p.second;
    Symbol s(n->getHash());
    pi.addSymbol(s);

    if (!n->isLeaf()) {
      worklist.push_back(std::make_pair(n->left, pi));
      worklist.push_back(std::make_pair(n->right, pi));
    } else {
      if (ids.find(n->stateID) != ids.end()) {
        handleLeaf(n, pi, matches);
      }
    }
  }

  /* try to match the non-repetitive patterns */
  unifyMatches(matches, result);
  for (PatternMatch &pm : result) {
    /* we sort, to help in finding distinct terms */
    /* TODO: something more efficient? */
    std::sort(pm.matches.begin(), pm.matches.end(), compare);
  }

  //for (PatternMatch &pm : result) {
  //  pm.dump();
  //}
}

void klee::traverse(ExecTree &t,
                    std::set<uint32_t> &ids,
                    std::vector<TreePath> &result) {
  std::vector<std::pair<ExecTreeNode *, Word>> worklist;
  worklist.push_back(std::make_pair(t.root, Word()));

  while (!worklist.empty()) {
    auto p = worklist.back();
    worklist.pop_back();

    ExecTreeNode *n = p.first;
    Word w = p.second;
    Symbol s(n->getHash());
    w.append(s);

    if (n->isLeaf()) {
      if (ids.find(n->stateID) != ids.end()) {
        result.push_back(TreePath(w, n->stateID));
      }
    } else {
      if (n->left) {
        worklist.push_back(std::make_pair(n->left, w));
      }
      if (n->right) {
        worklist.push_back(std::make_pair(n->right, w));
      }
    }
  }
}

void klee::extractPatternsBackward(ExecTree &t,
                                   std::set<uint32_t> &ids,
                                   std::vector<PatternMatch> &result) {
  std::vector<TreePath> paths;
  traverse(t, ids, paths);

  /* compute pattern instances */
  std::vector<PatternMatch> matches;
  for (TreePath &path : paths) {
    PatternInstance pi = PatternInstance();
    const Word &w = path.w;
    size_t n = path.w.size();
    for (unsigned i = 0; i < n; i++) {
      pi.addSymbol(w[n - i - 1]);
    }
    PatternInstance rpi = pi.reversed();
    addPattern(matches, rpi, path.stateID);
  }

  unifyMatches(matches, result);
  for (PatternMatch &pm : result) {
    /* TODO: something more efficient? */
    std::sort(pm.matches.begin(), pm.matches.end(), compare);
  }

  //for (PatternMatch &pm : result) {
  //  pm.dump();
  //}
}
