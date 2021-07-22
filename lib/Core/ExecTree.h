#ifndef KLEE_EXEC_TREE_H
#define KLEE_EXEC_TREE_H

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

  /* TODO: should be private */
  ExecTreeNode(std::uint32_t stateID,
               ref<Expr> e,
               ExecutionState *snapshot,
               std::uint32_t salt = 1);

  ExecTreeNode(const ExecTreeNode &other) :
    stateID(other.stateID),
    e(other.e),
    snapshot(other.snapshot),
    left(other.left),
    right(other.right),
    parent(other.parent),
    treeHash(other.treeHash),
    salt(other.salt) {

  }

public:

  ~ExecTreeNode();

  bool isLeaf() {
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

  uint32_t getHash() {
    return e->shapeHash() ^ salt;
  }

  uint32_t getTreeHash() {
    return treeHash;
  }

  uint32_t getSubTreeHash() {
    if (isLeaf()) {
      return 0;
    } else {
      /* excluding the root */
      return left->getTreeHash() ^ right->getTreeHash();
    }
  }

  std::uint32_t stateID;
  ref<Expr> e;
  ExecutionState *snapshot;
  ExecTreeNode *left;
  ExecTreeNode *right;
  ExecTreeNode *parent;
  /* TODO: remove */
  unsigned treeHash;
  std::uint32_t salt;
};

class ExecTree {
public:

  ExecTree(std::uint32_t stateID);

  ~ExecTree();

  ExecTree(const ExecTree &other);

  void extend(ExecutionState &current,
              ExecutionState &trueState,
              ExecutionState *trueSnapshot,
              ExecutionState &falseState,
              ExecutionState *falseSnapshot,
              ref<Expr> condition,
              std::uint32_t salt);

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

  void computeHashes();

  void computeNodeHashes(ExecTreeNode *n);

  ExecTreeNode *getNearestAncestor(ExecTreeNode *n1, ExecTreeNode *n2);

  void getReachable(ExecTreeNode *src,
                    std::vector<ExecTreeNode *> &reachable);

  ref<Expr> getPC(ExecTreeNode *from, ExecTreeNode *to);

  void removePathTo(ExecTreeNode *dst);

  void clear();

  void dump();

  void dumpGML(llvm::raw_ostream &os, std::set<uint32_t> &ids);

  void dumpGMLToFile(std::set<uint32_t> &ids, const std::string &name);

private:

  void addNode(ExecTreeNode *node);

  void removeNode(ExecTreeNode *node);

  void setLeft(ExecTreeNode *parent, ExecTreeNode *node);

  void setRight(ExecTreeNode *parent, ExecTreeNode *node);

public:

  ExecTreeNode *root;
  std::set<ExecTreeNode *> nodes;
};

}

#endif
