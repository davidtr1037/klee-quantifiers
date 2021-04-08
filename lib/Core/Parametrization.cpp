#include "Parametrization.h"
#include "ExecTreeIterator.h"

using namespace llvm;
using namespace klee;

void klee::extractEquationsForCore(ExecTree &t,
                                   PatternMatch &pm,
                                   std::vector<SMTEquationSystem> &result) {
  std::vector<SMTEquationSystem> systems(pm.pattern.core.size());

  /* find max k */
  unsigned max_k = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max_k) {
      max_k = sm.count;
    }
  }

  ExecTreeIterator iter(t);

  /* traverse prefix */
  for (unsigned i = 0; i < pm.pattern.prefix.size(); i++) {
    assert(iter.hasNext());
    iter.next(pm.pattern.prefix[i]);
  }

  /* traverse core */
  for (unsigned k = 0; k < max_k; k++) {
    for (unsigned i = 0; i < pm.pattern.core.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.core[i]);

      ref<Expr> e = iter.getCurrent()->e;
      systems[i].push_back(SMTEquation(e, k));
    }
  }

  /* TODO: avoid copy */
  for (SMTEquationSystem &es : systems) {
    result.push_back(es);
  }
}

void klee::extractEquationsForSuffix(ExecTree &t,
                                     PatternMatch &pm,
                                     std::vector<SMTEquationSystem> &result) {
  std::vector<SMTEquationSystem> systems(pm.pattern.suffix.size());

  for (StateMatch &sm : pm.matches) {
    ExecTreeIterator iter(t);

    /* traverse prefix */
    for (unsigned i = 0; i < pm.pattern.prefix.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.prefix[i]);
    }

    /* traverse core */
    for (unsigned k = 0; k < sm.count; k++) {
      for (unsigned i = 0; i < pm.pattern.core.size(); i++) {
        assert(iter.hasNext());
        iter.next(pm.pattern.core[i]);
      }
    }

    /* traverse suffix */
    for (unsigned i = 0; i < pm.pattern.suffix.size(); i++) {
      assert(iter.hasNext());
      iter.next(pm.pattern.suffix[i]);

      ref<Expr> e = iter.getCurrent()->e;
      systems[i].push_back(SMTEquation(e, sm.count));
    }
  }

  /* TODO: avoid copy */
  for (SMTEquationSystem &es : systems) {
    result.push_back(es);
  }
}
