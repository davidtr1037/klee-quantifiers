#ifndef KLEE_EXEC_TREE_H
#define KLEE_EXEC_TREE_H

#include "PTree.h"

#include <klee/Expr/Expr.h>

#include <stdint.h>
#include <vector>

namespace klee {

class ExecutionState;

/* TODO: StateID2Value */
typedef std::map<std::uint32_t, ref<Expr>> State2Value;

class ExecTreeNode {

friend class ExecTree;

private:

  ExecTreeNode(std::uint32_t stateID,
               ref<Expr> e,
               ExecutionState *snapshot,
               PTreeNode *ptreeNode,
               std::uint32_t salt = 1);

  ExecTreeNode(const ExecTreeNode &other) :
    stateID(other.stateID),
    e(other.e),
    snapshots(other.snapshots),
    ptreeNode(other.ptreeNode),
    left(other.left),
    right(other.right),
    parent(other.parent),
    isJoined(other.isJoined),
    salt(other.salt) {

  }

public:

  ~ExecTreeNode();

  bool isLeaf() const {
    return left == nullptr && right == nullptr;
  }

  ExecTreeNode *getSibling() const {
    if (!parent) {
      return nullptr;
    }

    if (parent->left == this) {
      return parent->right;
    } else {
      return parent->left;
    }
  }

  bool hasSibling() const {
    return getSibling() != nullptr;
  }

  uint32_t getHash() const {
    return e->shapeHash() ^ salt;
  }

  bool isComplete() const {
    return left && right;
  }

  std::uint32_t stateID;
  ref<Expr> e;
  std::vector<ExecutionState *> snapshots;
  PTreeNode *ptreeNode;
  ExecTreeNode *left;
  ExecTreeNode *right;
  ExecTreeNode *parent;
  bool isJoined;
  std::uint32_t salt;
};

/* TODO: move callbacks to another place? */

typedef uint32_t (*HashCallback)(ExecTreeNode *node);

uint32_t shapeHashCallback(ExecTreeNode *node);

uint32_t customHashCallback(ExecTreeNode *node);

class ExecTree {
public:

  ExecTree(std::uint32_t stateID);

  ~ExecTree();

  void setHashCallback(HashCallback callback);

  std::uint32_t getNodeHash(ExecTreeNode *node) const;

  void extend(ExecutionState &current,
              ExecutionState *trueState,
              ExecutionState *falseState,
              ref<Expr> condition,
              std::uint32_t salt);

  void addSnapshot(ExecutionState &state,
                   ExecutionState *snapshot);

  void setLeft(ExecTreeNode *parent,
               ExecutionState &state,
               ref<Expr> condition,
               ExecutionState *snapshot,
               std::uint32_t salt = 1);

  void setRight(ExecTreeNode *parent,
                ExecutionState &state,
                ref<Expr> condition,
                ExecutionState *snapshot,
                std::uint32_t salt = 1);

  ExecTreeNode *getNearestAncestor(ExecTreeNode *n1, ExecTreeNode *n2) const;

  void getReachable(ExecTreeNode *src,
                    std::vector<ExecTreeNode *> &reachable) const;

  ref<Expr> getPC(ExecTreeNode *parent,
                  ExecTreeNode *child,
                  bool inclusive = false) const;

  void removeSubTree(ExecTreeNode *dst,
                     ExecTreeNode *stopAt);

  bool join(ExecTreeNode *dst);

  bool isSinglePath(ExecTreeNode *node, ExecTreeNode *ancestor);

  void clear();

  void dump();

  void dumpGML(llvm::raw_ostream &os, std::set<uint32_t> &ids);

  void dumpGMLToFile(std::set<uint32_t> &ids, const std::string &name);

  void dumpGMLToFile(const std::string &name) {
    std::set<uint32_t> ids;
    dumpGMLToFile(ids, name);
  }

private:

  void addNode(ExecTreeNode *node);

  void removeNode(ExecTreeNode *node);

  void setLeft(ExecTreeNode *parent, ExecTreeNode *node);

  void setRight(ExecTreeNode *parent, ExecTreeNode *node);

  HashCallback hashCallback;

public:

  ExecTreeNode *root;
  std::set<ExecTreeNode *> nodes;
  std::set<ExecTreeNode *> nodesToMerge;
};

}

#endif
