#include "ExecTree.h"

using namespace std;
using namespace llvm;

namespace klee {

ExecTree::ExecTree(uint32_t stateID) {
  ExecTreeNode *n = new ExecTreeNode(stateID, ConstantExpr::create(1, Expr::Bool));
  root = n;
  addNode(n);
}

ExecTree::~ExecTree() {
  for (ExecTreeNode *n : nodes) {
    delete n;
  }
}

void ExecTree::addNode(ExecTreeNode *node) {
  nodes.push_back(node);
}

void ExecTree::extend(uint32_t stateID,
                      ref<Expr> condition,
                      uint32_t trueStateID,
                      uint32_t falseStateID) {
  ExecTreeNode *left = new ExecTreeNode(falseStateID, Expr::createIsZero(condition));
  ExecTreeNode *right = new ExecTreeNode(trueStateID, condition);

  for (ExecTreeNode *node : nodes) {
    if (node->stateID == stateID && node->isLeaf()) {
      node->left = left;
      node->right = right;
      addNode(left);
      addNode(right);
      return;
    }
  }

  assert(false);
}

void ExecTree::dump() {
  std::vector<ExecTreeNode *> worklist;
  worklist.push_back(root);

  while (!worklist.empty()) {
    ExecTreeNode *n = worklist.back();
    worklist.pop_back();

    errs() << "ID: " << n->stateID << " (leaf = " << n->isLeaf() << ")\n";
    n->e->dump();

    if (!n->isLeaf()) {
      worklist.push_back(n->left);
      worklist.push_back(n->right);
    }
  }
}

}