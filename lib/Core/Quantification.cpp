#include "Quantification.h"
#include "klee/Expr/ExprUtil.h"

#include "llvm/ADT/StringExtras.h"

#include <vector>
#include <list>

using namespace klee;
using namespace llvm;

cl::opt<unsigned> MinStateForABV(
    "min-states-for-abv",
    cl::init(1),
    cl::desc(""));

bool getParametricExpressions(std::vector<SMTEquationSystem> systems,
                              TimingSolver &solver,
                              uint32_t mergeID,
                              std::vector<ParametrizedExpr> &result) {
  for (SMTEquationSystem &system : systems) {
    ParametrizedExpr solution;
    if (!solveEquationSystem(system, solver, mergeID, solution)) {
      return false;
    }

    result.push_back(solution);
  }

  return true;
}

ref<Expr> generateForallPremise(ref<Expr> bound, ref<Expr> parameter) {
  return AndExpr::create(
    UleExpr::create(ConstantExpr::create(1, parameter->getWidth()), bound),
    UleExpr::create(bound, parameter)
  );
}

ref<Expr> generateRangeConstraint(PatternMatch &pm, ref<Expr> aux) {
  unsigned min = -1, max = 0;
  for (StateMatch &sm : pm.matches) {
    if (sm.count >= max) {
      max = sm.count;
    }
    if (sm.count <= min) {
      min = sm.count;
    }
  }

  ref<Expr> maxExpr = ConstantExpr::create(max, aux->getWidth());
  ref<Expr> minExpr = ConstantExpr::create(min, aux->getWidth());

  if (max == min) {
    /* there is only one path */
    return EqExpr::create(aux, maxExpr);
  }

  ref<Expr> range = AndExpr::create(
    UgeExpr::create(aux, minExpr),
    UleExpr::create(aux, maxExpr)
  );

  if ((max - min + 1) != pm.matches.size()) {
    for (unsigned i = min; i < max; i++) {
      bool missing = true;
      for (StateMatch &sm : pm.matches) {
        if (i == sm.count) {
          missing = false;
          break;
        }
      }
      if (missing) {
        ref<Expr> countExpr = ConstantExpr::create(i, aux->getWidth());
        range = AndExpr::create(
          range,
          Expr::createIsZero(EqExpr::create(aux, countExpr))
        );
      }
    }
  }

  return range;
}

void getKnownRangeValues(PatternMatch &pm,
                         std::vector<uint64_t> &range) {
  unsigned min = -1;
  for (StateMatch &sm : pm.matches) {
    if (sm.count <= min) {
      min = sm.count;
    }
  }

  for (unsigned i = 1; i < min + 1; i++) {
    range.push_back(i);
  }
}

void generateForall(PatternMatch &pm,
                    std::vector<ParametrizedExpr> solutions,
                    uint32_t mergeID,
                    ref<Expr> &forallExpr,
                    ref<Expr> &rangeExpr) {
  if (solutions.empty()) {
    forallExpr = rangeExpr = ConstantExpr::create(1, Expr::Bool);
    return;
  }

  unsigned auxArraySize = QuantifiedExpr::AUX_VARIABLE_SIZE;
  const Array *array_i = getArray("__i", auxArraySize, true, false);
  /* TODO: reuse the array from other module */
  const Array *array_m = getArrayForAuxVariable(mergeID, auxArraySize);

  /* the introduced variable */
  ref<Expr> aux = getSymbolicValue(array_m, array_m->size);
  /* outside of the forall expression */
  rangeExpr = generateRangeConstraint(pm, aux);

  ref<Expr> bound = getSymbolicValue(array_i, array_i->size);
  ref<Expr> premise = generateForallPremise(bound, aux);

  ref<Expr> coreExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : solutions) {
    /* TODO: rename */
    ref<Expr> i = getSymbolicValue(array_i, pe.parameter->getWidth() / 8);
    /* TODO: use the fixed visitor */
    ExprReplaceVisitor visitor(pe.parameter, i);
    /* TODO: rename */
    ref<Expr> substituted = visitor.visit(pe.e);
    coreExpr = AndExpr::create(coreExpr, substituted);
  }

  /* TODO: remove? */
  std::vector<uint64_t> range;
  getKnownRangeValues(pm, range);

  /* the forall expression itself */
  forallExpr = ForallExpr::create(
    bound,
    premise,
    coreExpr,
    range
  );
}

/* TODO: handle low number of matches */
ref<Expr> klee::generateQuantifiedConstraint(PatternMatch &pm,
                                             ExecTree &tree,
                                             uint32_t mergeID,
                                             TimingSolver &solver) {
  std::vector<SMTEquationSystem> coreSystems, suffixSystems;
  std::vector<ParametrizedExpr> coreSolutions, suffixSolutions;

  if (pm.matches.size() < MinStateForABV) {
    return nullptr;
  }

  ref<Expr> prefix = extractPrefixConstraint(tree, pm);

  extractEquationsForCore(tree, pm, coreSystems);
  if (!getParametricExpressions(coreSystems, solver, mergeID, coreSolutions)) {
    return nullptr;
  }

  extractEquationsForSuffix(tree, pm, suffixSystems);
  if (!getParametricExpressions(suffixSystems, solver, mergeID, suffixSolutions)) {
    return nullptr;
  }

  ref<Expr> forallExpr, rangeExpr;
  generateForall(pm, coreSolutions, mergeID, forallExpr, rangeExpr);

  ref<Expr> suffixExpr = ConstantExpr::create(1, Expr::Bool);
  for (const ParametrizedExpr &pe : suffixSolutions) {
    suffixExpr = AndExpr::create(suffixExpr, pe.e);
  }

  return AndExpr::create(
    prefix,
    AndExpr::create(
      rangeExpr,
      AndExpr::create(
        forallExpr,
        suffixExpr
      )
    )
  );
}

bool klee::generateMergedValue(PatternMatch &pm,
                               ExecTree &tree,
                               State2Value &valuesMap,
                               uint32_t mergeID,
                               TimingSolver &solver,
                               ParametrizedExpr &solution) {
  SMTEquationSystem system;
  extractEquationsForValue(tree, pm, valuesMap, system);

  if (!solveEquationSystem(system, solver, mergeID, solution)) {
    return false;
  }

  return true;
}

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

static ref<Expr> computeBoundTerm(ref<Expr> e, ref<Expr> target) {
  if (e->getWidth() != target->getWidth()) {
    return nullptr;
  }

  if (isBoundVariable(e)) {
    /* i == target */
    return target;
  }

  if (isa<AddExpr>(e)) {
    ref<AddExpr> addExpr = dyn_cast<AddExpr>(e);
    if (isBoundVariable(addExpr->left)) {
      /* TODO: make sure i doesn't appear in e */
      /* i + e = target */
      return SubExpr::create(target, addExpr->right);
    } else if (isBoundVariable(addExpr->right)) {
      /* TODO: make sure i doesn't appear in e */
      /* e + i = target */
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
  /* look for: not(eq e select(...)) */
  if (!isNegated) {
    return;
  }

  ref<ReadExpr> r = dyn_cast<ReadExpr>(assertion->right);
  if (r.isNull()) {
    return;
  }

  /* look for a store: [index = e] */
  unsigned seenStores = 0;
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
