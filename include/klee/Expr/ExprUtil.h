//===-- ExprUtil.h ----------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXPRUTIL_H
#define KLEE_EXPRUTIL_H

#include "klee/Expr/ExprVisitor.h"

#include <vector>

namespace klee {
  class Array;
  class Expr;
  class ReadExpr;
  template<typename T> class ref;

  /// Find all ReadExprs used in the expression DAG. If visitUpdates
  /// is true then this will including those reachable by traversing
  /// update lists. Note that this may be slow and return a large
  /// number of results.
  void findReads(ref<Expr> e, 
                 bool visitUpdates,
                 std::vector< ref<ReadExpr> > &result);
  
  /// Return a list of all unique symbolic objects referenced by the given
  /// expression.
  void findSymbolicObjects(ref<Expr> e,
                           std::vector<const Array*> &results);

  /// Return a list of all unique symbolic objects referenced by the
  /// given expression range.
  template<typename InputIterator>
  void findSymbolicObjects(InputIterator begin, 
                           InputIterator end,
                           std::vector<const Array*> &results);

  class ConstantArrayFinder : public ExprVisitor {
  protected:
    ExprVisitor::Action visitRead(const ReadExpr &re);

  public:
    std::set<const Array *> results;
  };

  class ExprReplaceVisitor : public ExprVisitor {
  private:
    ref<Expr> src, dst;

  public:
    ExprReplaceVisitor(const ref<Expr> &src, const ref<Expr> &dst)
        : src(src), dst(dst) {}

    Action visitExpr(const Expr &e) override {
      if (e == *src) {
        return Action::changeTo(dst);
      }
      return Action::doChildren();
    }

    Action visitExprPost(const Expr &e) override {
      if (e == *src) {
        return Action::changeTo(dst);
      }
      return Action::doChildren();
    }
  };

  class ExprReplaceVisitor2 : public ExprVisitor {
  private:
    const std::map< ref<Expr>, ref<Expr> > &replacements;

  public:
    explicit ExprReplaceVisitor2(
        const std::map<ref<Expr>, ref<Expr>> &_replacements)
        : ExprVisitor(true), replacements(_replacements) {}

    Action visitExprPost(const Expr &e) override {
      auto it = replacements.find(ref<Expr>(const_cast<Expr *>(&e)));
      if (it!=replacements.end()) {
        return Action::changeTo(it->second);
      }
      return Action::doChildren();
    }
  };

}

#endif /* KLEE_EXPRUTIL_H */
