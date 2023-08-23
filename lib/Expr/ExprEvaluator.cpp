//===-- ExprEvaluator.cpp -------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Expr/ExprEvaluator.h"
#include "klee/Expr/ExprUtil.h"

using namespace klee;

ExprVisitor::Action ExprEvaluator::evalRead(const UpdateList &ul,
                                            unsigned index) {
  for (auto un = ul.head; !un.isNull(); un = un->next) {
    ref<Expr> ui = visit(un->index);
    
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(ui)) {
      if (CE->getZExtValue() == index)
        return Action::changeTo(visit(un->value));
    } else {
      // update index is unknown, so may or may not be index, we
      // cannot guarantee value. we can rewrite to read at this
      // version though (mostly for debugging).
      
      return Action::changeTo(ReadExpr::create(UpdateList(ul.root, un), 
                                               ConstantExpr::alloc(index, 
                                                                   ul.root->getDomain())));
    }
  }
  
  if (ul.root->isConstantArray() && index < ul.root->size)
    return Action::changeTo(ul.root->constantValues[index]);

  return Action::changeTo(getInitialValue(*ul.root, index));
}

ExprVisitor::Action ExprEvaluator::visitExpr(const Expr &e) {
  // Evaluate all constant expressions here, in case they weren't folded in
  // construction. Don't do this for reads though, because we want them to go to
  // the normal rewrite path.
  unsigned N = e.getNumKids();
  if (!N || isa<ReadExpr>(e))
    return Action::doChildren();

  for (unsigned i = 0; i != N; ++i)
    if (!isa<ConstantExpr>(e.getKid(i)))
      return Action::doChildren();

  ref<Expr> Kids[3];
  for (unsigned i = 0; i != N; ++i) {
    assert(i < 3);
    Kids[i] = e.getKid(i);
  }

  return Action::changeTo(e.rebuild(Kids));
}

ExprVisitor::Action ExprEvaluator::visitRead(const ReadExpr &re) {
  ref<Expr> v = visit(re.index);
  
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(v)) {
    return evalRead(re.updates, CE->getZExtValue());
  } else {
    return Action::doChildren();
  }
}

// we need to check for div by zero during partial evaluation,
// if this occurs then simply ignore the 0 divisor and use the
// original expression.
ExprVisitor::Action ExprEvaluator::protectedDivOperation(const BinaryExpr &e) {
  ref<Expr> kids[2] = { visit(e.left),
                        visit(e.right) };

  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(kids[1]))
    if (CE->isZero())
      kids[1] = e.right;

  if (kids[0]!=e.left || kids[1]!=e.right) {
    return Action::changeTo(e.rebuild(kids));
  } else {
    return Action::skipChildren();
  }
}

ExprVisitor::Action ExprEvaluator::visitUDiv(const UDivExpr &e) { 
  return protectedDivOperation(e); 
}
ExprVisitor::Action ExprEvaluator::visitSDiv(const SDivExpr &e) { 
  return protectedDivOperation(e); 
}
ExprVisitor::Action ExprEvaluator::visitURem(const URemExpr &e) { 
  return protectedDivOperation(e); 
}
ExprVisitor::Action ExprEvaluator::visitSRem(const SRemExpr &e) { 
  return protectedDivOperation(e); 
}

ExprVisitor::Action ExprEvaluator::visitExprPost(const Expr& e) {
  // When evaluating an assignment we should fold NotOptimizedExpr
  // nodes so we can fully evaluate.
  if (e.getKind() == Expr::NotOptimized) {
    return Action::changeTo(static_cast<const NotOptimizedExpr&>(e).src);
  }
  return Action::skipChildren();
}

ExprVisitor::Action ExprEvaluator::visitForall(const ForallExpr &e) {
  /* can't handle nested quantifiers */
  assert(e.post->isQF);

  ref<AndExpr> pre = dyn_cast<AndExpr>(visit(e.pre));
  assert(!pre.isNull());
  ref<UleExpr> lower = dyn_cast<UleExpr>(pre->left);
  assert(!lower.isNull());
  ref<UleExpr> upper = dyn_cast<UleExpr>(pre->right);
  assert(!upper.isNull());

  /* get the bound variable */
  ref<Expr> var = lower->right;

  ref<ConstantExpr> minExpr = dyn_cast<ConstantExpr>(lower->left);
  ref<ConstantExpr> maxExpr = dyn_cast<ConstantExpr>(upper->right);
  assert(!minExpr.isNull());
  assert(!maxExpr.isNull());
  uint64_t min = minExpr->getZExtValue();
  uint64_t max = maxExpr->getZExtValue();

  ref<Expr> post = visit(e.post);
  std::vector<ref<ReadExpr>> reads;
  findReads(post, true, reads);

  ref<Expr> result = ConstantExpr::create(1, Expr::Bool);
  for (uint64_t i = min; i <= max; i++) {
    std::map<ref<Expr>, ref<Expr>> map;
    for (ref<ReadExpr> e : reads) {
      /* TODO: check if it's an array of a bound variable... */
      if (e->updates.root->isBoundVariable) {
        ref<ConstantExpr> index = dyn_cast<ConstantExpr>(e->index);
        assert(!index.isNull());
        uint64_t off = index->getZExtValue();
        map[e] = ConstantExpr::create((i >> (8 * off)) & 0xff, Expr::Int8);
      }
    }

    ExprReplaceVisitor2 visitor(map);
    ref<Expr> substituted = visitor.visit(post);
    if (!isa<ConstantExpr>(substituted)) {
      substituted = visit(substituted);
    }

    ref<ConstantExpr> sat = dyn_cast<ConstantExpr>(substituted);
    assert(!sat.isNull());

    /* TODO: remove AndExpr */
    result = AndExpr::create(result, sat);
    if (sat->isFalse()) {
      break;
    }
  }

  return Action::changeTo(result);
}
