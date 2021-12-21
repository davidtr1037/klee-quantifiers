#include "klee/Expr/EMatching.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprUtil.h"

#include <vector>
#include <list>

using namespace klee;
using namespace llvm;

static void findAssertions(ref<ForallExpr> f,
                           std::vector<EqAssertion> &assertions) {
  std::list<ref<Expr>> worklist;
  worklist.push_front(f->post);
  while (!worklist.empty()) {
    ref<Expr> e = worklist.front();
    worklist.pop_front();

    if (isa<AndExpr>(e)) {
      ref<AndExpr> andExpr = dyn_cast<AndExpr>(e);
      worklist.push_front(andExpr->right);
      worklist.push_front(andExpr->left);
    } else if (isa<EqExpr>(e)) {
      ref<EqExpr> eqExpr = dyn_cast<EqExpr>(e);
      if (eqExpr->left->getWidth() == Expr::Bool && eqExpr->left->isFalse()) {
        if (isa<EqExpr>(eqExpr->right)) {
          assertions.push_back(
            EqAssertion(dyn_cast<EqExpr>(eqExpr->right), true)
          );
        }
      } else {
        assertions.push_back(EqAssertion(eqExpr, false));
      }
    }
  }
}

static bool isBoundVariable(ref<Expr> e) {
  if (isa<ReadExpr>(e)) {
    return dyn_cast<ReadExpr>(e)->updates.root->isBoundVariable;
  }

  if (isa<ConcatExpr>(e)) {
    /* TODO: check the right expression */
    ref<ConcatExpr> c = dyn_cast<ConcatExpr>(e);
    return isBoundVariable(c->getLeft());
  }

  return false;
}

/* e depends on a bound variable */
static ref<Expr> computeBoundTerm(ref<Expr> e, ref<Expr> target) {
  if (e->getWidth() != target->getWidth()) {
    return nullptr;
  }

  if (target->hasBoundVariable) {
    /* TODO: add support? */
    return nullptr;
  }

  if (isBoundVariable(e)) {
    /* i == target */
    return target;
  }

  if (isa<AddExpr>(e)) {
    ref<AddExpr> addExpr = dyn_cast<AddExpr>(e);
    if (isBoundVariable(addExpr->left)) {
      /* i + e = target */
      if (addExpr->right->hasBoundVariable) {
        /* e depends on i */
        return nullptr;
      }
      return SubExpr::create(target, addExpr->right);
    } else if (isBoundVariable(addExpr->right)) {
      /* e + i = target */
      if (addExpr->left->hasBoundVariable) {
        /* e depends on i */
        return nullptr;
      }
      return SubExpr::create(target, addExpr->left);
    } else {
      /* TODO: check the other case? */
      return computeBoundTerm(
        addExpr->right,
        SubExpr::create(target, addExpr->left)
      );
    }
  }

  if (isa<ExtractExpr>(e)) {
    /* extract(e, w) == target */
    ref<ExtractExpr> extractExpr = dyn_cast<ExtractExpr>(e);
    return computeBoundTerm(
      extractExpr->expr,
      ZExtExpr::create(target, extractExpr->expr->getWidth())
    );
  }

  return nullptr;
}

void EqAssertion::findImpliedNegatingTerms(std::vector<ref<Expr>> &terms) {
  if (!isNegated) {
    /* TODO: support this case (although seems to be less common) */
    return;
  }

  ref<ReadExpr> r = dyn_cast<ReadExpr>(assertion->right);
  if (r.isNull()) {
    return;
  }

  /* the case: not(eq e select(...)) */
  unsigned seenStores = 0;
  /* look for a store: [index = e] */
  for (ref<UpdateNode> un = r->updates.head; !un.isNull(); un = un->next) {
    if (*un->value == *assertion->left) {
      if (seenStores == 0) {
        ref<Expr> term = computeBoundTerm(r->index, un->index);
        if (!term.isNull()) {
          terms.push_back(term);
        }
      }
      break;
    }
    seenStores++;
  }
}

void EqAssertion::findNegatingTermsForRead(ref<ReadExpr> p,
                                           ref<ReadExpr> e,
                                           std::vector<ref<Expr>> &terms) {
  if (p->updates.compare(e->updates) != 0) {
    return;
  }

  /* find a term t such that: p(i)[t/i] = e */
  ref<Expr> term = computeBoundTerm(p->index, e->index);
  if (!term.isNull()) {
    terms.push_back(term);
  }
}

/* TODO: should be findNegatingTerm? */
void EqAssertion::findNegatingTerms(ref<Expr> e,
                                    std::vector<ref<Expr>> &terms) {
  ref<EqExpr> eq = dyn_cast<EqExpr>(e);
  if (eq.isNull()) {
    return;
  }

  /* check if: (Eq false ...) */
  if (eq->left->getWidth() == Expr::Bool && eq->left->isFalse()) {
    if (isNegated) {
      return;
    }

    /* check if: (Eq false (Eq ...)) */
    eq = dyn_cast<EqExpr>(eq->right);
    if (eq.isNull()) {
      return;
    }
  } else {
    /* the case: (Eq ... ...) */
    if (!isNegated) {
      return;
    }
  }

  /* TODO: check the case where the RHS is equal? */
  if (*assertion->left != *eq->left) {
    return;
  }

  if (isa<ReadExpr>(assertion->right) && isa<ReadExpr>(eq->right)) {
    findNegatingTermsForRead(dyn_cast<ReadExpr>(assertion->right),
                             dyn_cast<ReadExpr>(eq->right),
                             terms);
  }
}

void EqAssertion::findNegatingTerms(const ConstraintSet &constraints,
                                    bool checkImplied,
                                    bool checkConstraints,
                                    std::vector<ref<Expr>> &terms) {
  if (checkImplied) {
    findImpliedNegatingTerms(terms);
  }
  if (checkConstraints) {
    for (ref<Expr> constraint : constraints) {
      findNegatingTerms(constraint, terms);
    }
  }
}

void klee::generateLemmaFromForall(ref<ForallExpr> f,
                                   ConstraintSet &constraints,
                                   bool checkImplied,
                                   bool checkConstraints,
                                   std::vector<ref<Expr>> &lemmas) {
  /* TODO: any better way to handle? */
  if (!f->auxArray) {
    return;
  }
  std::vector<EqAssertion> assertions;
  findAssertions(f, assertions);

  std::vector<ref<Expr>> terms;
  for (EqAssertion &a : assertions) {
    a.findNegatingTerms(constraints, checkImplied, checkConstraints, terms);
  }

  ref<Expr> aux = getSymbolicValue(f->auxArray, f->auxArray->size);

  for (ref<Expr> term : terms) {
    term = ZExtExpr::create(term, QuantifiedExpr::AUX_VARIABLE_WIDTH);
    ref<Expr> lemma = OrExpr::create(
      EqExpr::create(ConstantExpr::create(0, term->getWidth()), aux),
      OrExpr::create(
        UltExpr::create(term, ConstantExpr::create(1, term->getWidth())),
        UltExpr::create(aux, term)
      )
    );
    lemmas.push_back(lemma);
  }
}

ref<Expr> klee::findForallExpr(ref<Expr> e) {
  std::list<ref<Expr>> worklist;
  worklist.push_back(e);
  while (!worklist.empty()) {
    ref<Expr> e = worklist.front();
    worklist.pop_front();
    if (isa<ForallExpr>(e)) {
      return dyn_cast<ForallExpr>(e);
    } else if (isa<AndExpr>(e)) {
      ref<AndExpr> andExpr = dyn_cast<AndExpr>(e);
      worklist.push_back(andExpr->left);
      worklist.push_back(andExpr->right);
    }
  }

  return nullptr;
}
