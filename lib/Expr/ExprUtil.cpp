//===-- ExprUtil.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Expr/ExprUtil.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprHashMap.h"
#include "klee/Expr/ExprVisitor.h"

#include <set>
#include <list>
#include <cmath>
#include <algorithm>

using namespace klee;

void klee::findReads(ref<Expr> e, 
                     bool visitUpdates,
                     std::vector< ref<ReadExpr> > &results) {
  // Invariant: \forall_{i \in stack} !i.isConstant() && i \in visited 
  std::vector< ref<Expr> > stack;
  ExprHashSet visited;
  std::set<const UpdateNode *> updates;
  
  if (!isa<ConstantExpr>(e)) {
    visited.insert(e);
    stack.push_back(e);
  }

  while (!stack.empty()) {
    ref<Expr> top = stack.back();
    stack.pop_back();

    if (ReadExpr *re = dyn_cast<ReadExpr>(top)) {
      // We memoized so can just add to list without worrying about
      // repeats.
      results.push_back(re);

      if (!isa<ConstantExpr>(re->index) &&
          visited.insert(re->index).second)
        stack.push_back(re->index);
      
      if (visitUpdates) {
        // XXX this is probably suboptimal. We want to avoid a potential
        // explosion traversing update lists which can be quite
        // long. However, it seems silly to hash all of the update nodes
        // especially since we memoize all the expr results anyway. So
        // we take a simple approach of memoizing the results for the
        // head, which often will be shared among multiple nodes.
        if (updates.insert(re->updates.head.get()).second) {
          for (const auto *un = re->updates.head.get(); un;
               un = un->next.get()) {
            if (!isa<ConstantExpr>(un->index) &&
                visited.insert(un->index).second)
              stack.push_back(un->index);
            if (!isa<ConstantExpr>(un->value) &&
                visited.insert(un->value).second)
              stack.push_back(un->value);
          }
        }
      }
    } else if (!isa<ConstantExpr>(top)) {
      Expr *e = top.get();
      for (unsigned i=0; i<e->getNumKids(); i++) {
        ref<Expr> k = e->getKid(i);
        if (!isa<ConstantExpr>(k) &&
            visited.insert(k).second)
          stack.push_back(k);
      }
    }
  }
}

///

namespace klee {

class SymbolicObjectFinder : public ExprVisitor {
protected:
  Action visitRead(const ReadExpr &re) {
    const UpdateList &ul = re.updates;

    // XXX should we memo better than what ExprVisitor is doing for us?
    for (const auto *un = ul.head.get(); un; un = un->next.get()) {
      visit(un->index);
      visit(un->value);
    }

    if (ul.root->isSymbolicArray())
      if (results.insert(ul.root).second)
        objects.push_back(ul.root);

    return Action::doChildren();
  }

public:
  std::set<const Array*> results;
  std::vector<const Array*> &objects;
  
  SymbolicObjectFinder(std::vector<const Array*> &_objects)
    : objects(_objects) {}
};

ExprVisitor::Action ConstantArrayFinder::visitRead(const ReadExpr &re) {
  const UpdateList &ul = re.updates;

  // FIXME should we memo better than what ExprVisitor is doing for us?
  for (const auto *un = ul.head.get(); un; un = un->next.get()) {
    visit(un->index);
    visit(un->value);
  }

  if (ul.root->isConstantArray()) {
    results.insert(ul.root);
  }

  return Action::doChildren();
}

}

template<typename InputIterator>
void klee::findSymbolicObjects(InputIterator begin, 
                               InputIterator end,
                               std::vector<const Array*> &results) {
  SymbolicObjectFinder of(results);
  for (; begin!=end; ++begin)
    of.visit(*begin);
}

void klee::findSymbolicObjects(ref<Expr> e,
                               std::vector<const Array*> &results) {
  findSymbolicObjects(&e, &e+1, results);
}

static ref<Expr> simplifyITEEqEq(ref<SelectExpr> e) {
  /* looking for an expression of the form:
   * ite(c1 == x, v, ite(c2 == x, v, ...)) */
  ref<SelectExpr> inner = dyn_cast<SelectExpr>(e->falseExpr);

  ref<EqExpr> eq1 = dyn_cast<EqExpr>(e->cond);
  ref<EqExpr> eq2 = dyn_cast<EqExpr>(inner->cond);
  if (eq1.isNull() || eq2.isNull()) {
    return nullptr;
  }

  if (*eq1->right != *eq2->right) {
    return nullptr;
  }

  ref<ConstantExpr> v1 = dyn_cast<ConstantExpr>(eq1->left);
  ref<ConstantExpr> v2 = dyn_cast<ConstantExpr>(eq2->left);
  if (v1.isNull() || v2.isNull()) {
    return nullptr;
  }

  uint64_t n1 = v1->getZExtValue();
  uint64_t n2 = v2->getZExtValue();
  if (abs(n1 - n2) != 1) {
    return nullptr;
  }

  uint64_t low = std::min(n1, n2);
  uint64_t high = std::max(n1, n2);
  if (low == 0 && high == (uint64_t)(-1)) {
    return nullptr;
  }

  ref<Expr> cond = AndExpr::create(
    UleExpr::create(
      ConstantExpr::create(low, eq1->right->getWidth()),
      eq1->right
    ),
    UleExpr::create(
      eq1->right,
      ConstantExpr::create(high, eq1->right->getWidth())
    )
  );
  return SelectExpr::create(cond, inner->trueExpr, inner->falseExpr);
}

static ref<Expr> simplifyITEEqRange(ref<SelectExpr> e) {
  /* looking for an expression of the form:
   * ite(x == c1, v, ite(c2 <= x and x <= c3, v, ...)) */
  ref<SelectExpr> inner = dyn_cast<SelectExpr>(e->falseExpr);

  ref<EqExpr> eq = dyn_cast<EqExpr>(e->cond);
  ref<AndExpr> range = dyn_cast<AndExpr>(inner->cond);
  if (eq.isNull() || range.isNull()) {
    return nullptr;
  }

  ref<UleExpr> ule1 = dyn_cast<UleExpr>(range->left);
  ref<UleExpr> ule2 = dyn_cast<UleExpr>(range->right);
  if (ule1.isNull() || ule2.isNull()) {
    return nullptr;
  }

  if ((*ule1->right != *ule2->left) || (*ule1->right != *eq->right)) {
    return nullptr;
  }

  ref<ConstantExpr> v = dyn_cast<ConstantExpr>(eq->left);
  ref<ConstantExpr> rv1 = dyn_cast<ConstantExpr>(ule1->left);
  ref<ConstantExpr> rv2 = dyn_cast<ConstantExpr>(ule2->right);
  if (v.isNull() || rv1.isNull() || rv2.isNull()) {
    return nullptr;
  }

  uint64_t n = v->getZExtValue();
  uint64_t rn1 = rv1->getZExtValue();
  uint64_t rn2 = rv2->getZExtValue();
  if (rn1 == n + 1 && n != (uint64_t)(-1)) {
    ref<Expr> cond = AndExpr::create(
      UleExpr::create(
        ConstantExpr::create(n, eq->right->getWidth()),
        eq->right
      ),
      UleExpr::create(
        eq->right,
        ConstantExpr::create(rn2, eq->right->getWidth())
      )
    );
    return SelectExpr::create(cond, inner->trueExpr, inner->falseExpr);
  }

  if (n == rn2 + 1 && rn2 != (uint64_t)(-1)) {
    ref<Expr> cond = AndExpr::create(
      UleExpr::create(
        ConstantExpr::create(rn1, eq->right->getWidth()),
        eq->right
      ),
      UleExpr::create(
        eq->right,
        ConstantExpr::create(n, eq->right->getWidth())
      )
    );
    return SelectExpr::create(cond, inner->trueExpr, inner->falseExpr);
  }

  return nullptr;
}

ref<Expr> klee::simplifyITE(ref<Expr> ite) {
  ref<SelectExpr> e = dyn_cast<SelectExpr>(ite);
  if (e.isNull()) {
    return ite;
  }

  ref<SelectExpr> inner = dyn_cast<SelectExpr>(e->falseExpr);
  if (inner.isNull()) {
    return e;
  }

  if (*e->trueExpr != *inner->trueExpr) {
    return e;
  }

  ref<Expr> simplified;
  simplified = simplifyITEEqEq(e);
  if (!simplified.isNull()) {
    return simplified;
  }

  simplified = simplifyITEEqRange(e);
  if (!simplified.isNull()) {
    return simplified;
  }

  return e;
}

typedef std::vector< ref<Expr> >::iterator A;
template void klee::findSymbolicObjects<A>(A, A, std::vector<const Array*> &);

typedef std::set< ref<Expr> >::iterator B;
template void klee::findSymbolicObjects<B>(B, B, std::vector<const Array*> &);

UpdateListCache ExprFullReplaceVisitorBase::cache;

ExprVisitor::Action ExprFullReplaceVisitorBase::visitRead(const ReadExpr &e) {
  ref<Expr> index = visit(e.index);

  UpdateList updates = UpdateList(nullptr, nullptr);
  auto i = cache.find(e.updates);
  if (i == cache.end()) {
    /* TODO: rewrite only if one of the nodes changes */
    updates = UpdateList(e.updates.root, nullptr);
    std::list<const UpdateNode *> nodes;
    for (const UpdateNode *n = e.updates.head.get(); n; n = n->next.get()) {
      nodes.push_front(n);
    }
    for (const UpdateNode *n : nodes) {
      ref<Expr> index = visit(n->index);
      ref<Expr> value = visit(n->value);
      updates.extend(index, value);
    }
    cache.insert(std::make_pair(e.updates, updates));
  } else {
    updates = i->second;
  }

  return Action::changeTo(ReadExpr::create(updates, index));
}
