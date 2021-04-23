#ifndef KLEE_EXEC_TREE_H
#define KLEE_EXEC_TREE_H

#include <klee/Expr/Expr.h>

#include <stdint.h>
#include <vector>

namespace klee {

/* TODO: StateID2Value */
typedef std::map<std::uint32_t, ref<Expr>> State2Value;

class ExecTreeNode {
public:

  ExecTreeNode(std::uint32_t stateID, ref<Expr> e, std::uint32_t salt = 1) :
    stateID(stateID), e(e), left(nullptr), right(nullptr), subTreeHash(0), salt(salt) {

  }

  bool isLeaf() {
    return left == nullptr && right == nullptr;
  }

  uint32_t getHash() {
    return e->shapeHash() ^ salt;
  }

  std::uint32_t stateID;
  ref<Expr> e;
  ExecTreeNode *left;
  ExecTreeNode *right;
  unsigned subTreeHash;
  std::uint32_t salt;
};

class ExecTree {
public:

  ExecTree(std::uint32_t stateID);

  ~ExecTree();

  void addNode(ExecTreeNode *node);

  void extend(std::uint32_t stateID,
              ref<Expr> condition,
              std::uint32_t leftID,
              std::uint32_t rightID,
              std::uint32_t salt);

  void computeHashes();

  void computeNodeHashes(ExecTreeNode *n);

  void dump();

  void dumpGML(llvm::raw_ostream &os, std::set<uint32_t> &ids);

  void dumpGMLToFile(std::set<uint32_t> &ids, std::string &name);

  ExecTreeNode *root;
  std::vector<ExecTreeNode *> nodes;
};

}

#endif
