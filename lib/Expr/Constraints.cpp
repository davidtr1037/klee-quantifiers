//===-- Constraints.cpp ---------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Expr/Constraints.h"

#include "klee/Expr/ExprVisitor.h"
#include "klee/Expr/ExprUtil.h"
#include "klee/Module/KModule.h"
#include "klee/Support/OptionCategories.h"

#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"

#include <map>

using namespace klee;

namespace {
llvm::cl::opt<bool> RewriteEqualities(
    "rewrite-equalities",
    llvm::cl::desc("Rewrite existing constraints when an equality with a "
                   "constant is added (default=true)"),
    llvm::cl::init(true),
    llvm::cl::cat(SolvingCat));

llvm::cl::opt<bool> ExtractForallEqualities(
    "extract-forall-equalities",
    llvm::cl::desc(""),
    llvm::cl::init(true),
    llvm::cl::cat(SolvingCat));
} // namespace

bool ConstraintManager::rewriteConstraints(ExprVisitor &visitor) {
  ConstraintSet old;
  bool changed = false;

  std::swap(constraints, old);
  for (auto &ce : old) {
    ref<Expr> e = visitor.visit(ce);

    if (e!=ce) {
      addConstraintInternal(e); // enable further reductions
      changed = true;
    } else {
      constraints.push_back(ce);
    }
  }

  return changed;
}

void ConstraintManager::extractEqualities(const ref<ForallExpr> &fe,
                                          ExprMap &equalities) {
  if (!isa<EqExpr>(fe->post)) {
    return;
  }

  std::vector<ref<ReadExpr>> reads;
  findReads(fe->post, true, reads);

  for (uint64_t i : fe->range) {
    std::map<ref<Expr>, ref<Expr>> map;
    for (ref<ReadExpr> e : reads) {
      if (e->updates.root->isBoundVariable) {
        ref<ConstantExpr> index = dyn_cast<ConstantExpr>(e->index);
        assert(!index.isNull());
        uint64_t off = index->getZExtValue();
        map[e] = ConstantExpr::create(i >> (8 * off), Expr::Int8);
      }
    }

    ExprReplaceVisitor2 visitor(map);
    ref<Expr> substituted = visitor.visit(fe->post);
    if (const EqExpr *eq = dyn_cast<EqExpr>(substituted)) {
      if (isa<ConstantExpr>(eq->left)) {
        equalities.insert(std::make_pair(eq->right, eq->left));
      }
    }
  }
}

ref<Expr> ConstraintManager::simplifyExpr(const ConstraintSet &constraints,
                                          const ref<Expr> &e) {

  if (isa<ConstantExpr>(e))
    return e;

  std::map< ref<Expr>, ref<Expr> > equalities;

  for (auto &constraint : constraints) {
    if (const EqExpr *ee = dyn_cast<EqExpr>(constraint)) {
      if (isa<ConstantExpr>(ee->left)) {
        equalities.insert(std::make_pair(ee->right,
                                         ee->left));
      } else {
        equalities.insert(
            std::make_pair(constraint, ConstantExpr::alloc(1, Expr::Bool)));
      }
    } else {
      equalities.insert(
          std::make_pair(constraint, ConstantExpr::alloc(1, Expr::Bool)));
    }

    if (ExtractForallEqualities && isa<ForallExpr>(constraint)) {
      extractEqualities(dyn_cast<ForallExpr>(constraint), equalities);
    }
  }

  return ExprReplaceVisitor2(equalities).visit(e);
}

void ConstraintManager::addConstraintInternal(const ref<Expr> &e) {
  // rewrite any known equalities and split Ands into different conjuncts

  switch (e->getKind()) {
  case Expr::Constant:
    assert(cast<ConstantExpr>(e)->isTrue() &&
           "attempt to add invalid (false) constraint");
    break;

    // split to enable finer grained independence and other optimizations
  case Expr::And: {
    BinaryExpr *be = cast<BinaryExpr>(e);
    addConstraintInternal(be->left);
    addConstraintInternal(be->right);
    break;
  }

  case Expr::Eq: {
    if (RewriteEqualities) {
      // XXX: should profile the effects of this and the overhead.
      // traversing the constraints looking for equalities is hardly the
      // slowest thing we do, but it is probably nicer to have a
      // ConstraintSet ADT which efficiently remembers obvious patterns
      // (byte-constant comparison).
      BinaryExpr *be = cast<BinaryExpr>(e);
      if (isa<ConstantExpr>(be->left)) {
	ExprReplaceVisitor visitor(be->right, be->left);
	rewriteConstraints(visitor);
      }
    }
    constraints.push_back(e);
    break;
  }

  default:
    constraints.push_back(e);
    break;
  }
}

void ConstraintManager::addConstraint(const ref<Expr> &e) {
  ref<Expr> simplified = simplifyExpr(constraints, e);
  addConstraintInternal(simplified);
}

void ConstraintManager::dump() const {
  constraints.dump();
}

ConstraintManager::ConstraintManager(ConstraintSet &_constraints)
    : constraints(_constraints) {}

bool ConstraintSet::empty() const { return constraints.empty(); }

klee::ConstraintSet::constraint_iterator ConstraintSet::begin() const {
  return constraints.begin();
}

klee::ConstraintSet::constraint_iterator ConstraintSet::end() const {
  return constraints.end();
}

size_t ConstraintSet::size() const noexcept { return constraints.size(); }

void ConstraintSet::push_back(const ref<Expr> &e) { constraints.push_back(e); }

void ConstraintSet::dump() const {
  for (ref<Expr> e : constraints) {
    e->dump();
  }
}
