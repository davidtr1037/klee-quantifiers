#include <klee/Expr/ExprRename.h>
#include <klee/Expr/Expr.h>
#include <klee/Expr/ExprUtil.h>
#include <klee/Expr/ExprHashMap.h>
#include <klee/Expr/ExprVisitor.h>
#include <klee/Expr/ArrayCache.h>

#include <llvm/ADT/StringExtras.h>

#include <set>

using namespace klee;

static ArrayCache cache;

const Array *klee::cloneArray(const Array *from, unsigned i) {
  const Array *cloned = cache.CreateArray(
    "p_" + llvm::utostr(i),
    from->size
  );
  /* TODO: refactor */
  cloned->isBoundVariable = from->isBoundVariable;
  cloned->isAuxVariable = from->isAuxVariable;
  return cloned;
}

ref<Expr> klee::rename(const ref<Expr> e, const ArrayMap &map) {
  if (!e->hasAuxVariable) {
    return e;
  }

  std::vector<ref<ReadExpr>> reads;
  findReads(e, true, reads);

  std::set<const Array *> arrays;
  for (ref<ReadExpr> e : reads) {
    assert(e->updates.root);
    if (e->updates.root->isAuxVariable) {
      arrays.insert(e->updates.root);
    }
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
  return visitor.visit(e);
}

void klee::rename(const Query &query,
                  ConstraintSet &constraints,
                  ref<Expr> &expr,
                  ArrayMap &map) {
  std::vector<ref<ReadExpr>> reads;
  if (query.expr->hasAuxVariable) {
    findReads(query.expr, true, reads);
  }
  for (ref<Expr> e : query.constraints) {
    if (e->hasAuxVariable) {
      findReads(e, true, reads);
    }
  }

  std::set<const Array *> arrays;
  for (ref<ReadExpr> e : reads) {
    assert(e->updates.root);
    if (e->updates.root->isAuxVariable) {
      arrays.insert(e->updates.root);
    }
  }

  /* sort arrays by id */
  for (const Array *array : arrays) {
    map[array] = nullptr;
  }
  unsigned index = 0;
  for (auto i : map) {
    const Array *array = i.first;
    map[array] = cloneArray(array, index);
    index++;
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
  expr = query.expr->hasAuxVariable ? visitor.visit(query.expr) : query.expr;
  for (ref<Expr> e : query.constraints) {
    constraints.push_back(e->hasAuxVariable ? visitor.visit(e) : e);
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

Query klee::rename(const Query &query) {
  ConstraintSet constraints;
  ref<Expr> expr;
  rename(query, constraints, expr);
  return Query(constraints, expr);
}

Query klee::rename(const Query &query,
                   const std::vector<const Array *> &objects,
                   std::vector<const Array *> &renamedObjects) {
  ConstraintSet constraints;
  ref<Expr> expr;
  rename(query, objects, constraints, expr, renamedObjects);
  return Query(constraints, expr);
}
