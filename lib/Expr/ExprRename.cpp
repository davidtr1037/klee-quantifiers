#include <klee/Expr/ExprRename.h>
#include <klee/Expr/Expr.h>
#include <klee/Expr/ExprUtil.h>
#include <klee/Expr/ExprHashMap.h>
#include <klee/Expr/ExprVisitor.h>
#include <klee/Expr/ArrayCache.h>

#include <llvm/ADT/StringExtras.h>

#include <set>

using namespace klee;

static ArrayCache arrayCache;

struct {
  bool operator()(const Array *a, const Array *b) const {
    return a->id < b->id;
  }
} ArrayCompare;

void klee::rename(const Query &query,
                  ConstraintSet &constraints,
                  ref<Expr> &expr,
                  ArrayMap &map) {
  std::vector<ref<ReadExpr>> reads;
  findReads(query.expr, true, reads);
  for (ref<Expr> e : query.constraints) {
    findReads(e, true, reads);
  }

  std::set<const Array *> arrays;
  for (ref<ReadExpr> e : reads) {
    assert(e->updates.root);
    if (e->updates.root->isAuxVariable) {
      arrays.insert(e->updates.root);
    }
  }

  std::vector<const Array *> sorted(arrays.begin(), arrays.end());
  std::sort(sorted.begin(), sorted.end(), ArrayCompare);

  for (unsigned i = 0; i < sorted.size(); i++) {
    const Array *array = sorted[i];
    const Array *newArray = arrayCache.CreateArray(
      "p_" + llvm::utostr(i),
      array->size
    );
    /* TODO: refactor */
    newArray->isBoundVariable = array->isBoundVariable;
    newArray->isAuxVariable = array->isAuxVariable;
    map[array] = newArray;
  }

  std::map<ref<Expr>, ref<Expr>> toReplace;
  for (ref<ReadExpr> e : reads) {
    if (e->updates.root && e->updates.root->isAuxVariable) {
      assert(isa<ConstantExpr>(e->index));
      auto i = map.find(e->updates.root);
      if (i == map.end()) {
        assert(0);
      } else {
        toReplace[e] = ReadExpr::create(UpdateList(i->second, 0), e->index);
      }
    }
  }

  ExprFullReplaceVisitor2 visitor(toReplace);
  expr = visitor.visit(query.expr);
  for (ref<Expr> e : query.constraints) {
    ref<Expr> x = visitor.visit(e);
    constraints.push_back(visitor.visit(e));
  }
}

void klee::rename(const Query &query,
                  ConstraintSet &constraints,
                  ref<Expr> &expr) {
  ArrayMap map;
  rename(query, constraints, expr, map);
}

void klee::rename(const Query &query,
                  const std::vector<const Array *> &objects,
                  ConstraintSet &constraints,
                  ref<Expr> &expr,
                  std::vector<const Array *> &renamedObjects) {
  ArrayMap map;
  rename(query, constraints, expr, map);
  for (const Array *array : objects) {
    auto i = map.find(array);
    if (i == map.end()) {
      renamedObjects.push_back(array);
    } else {
      renamedObjects.push_back(i->second);
    }
  }
}
