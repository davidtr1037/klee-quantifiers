#include "ExecTree.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"

#include <list>

using namespace std;
using namespace llvm;

namespace klee {

ExecTree::ExecTree(uint32_t stateID) {
  root = new ExecTreeNode(stateID, ConstantExpr::create(1, Expr::Bool));
  addNode(root);
}

ExecTree::~ExecTree() {
  for (ExecTreeNode *n : nodes) {
    delete n;
  }
}

ExecTree::ExecTree(const ExecTree &other) {
  root = new ExecTreeNode(*other.root);
  addNode(root);

  std::list<std::pair<ExecTreeNode *, ExecTreeNode *>> worklist;
  worklist.push_back(std::make_pair(other.root, root));
  while (!worklist.empty()) {
    auto p = worklist.front();
    worklist.pop_front();

    ExecTreeNode *other_n = p.first;
    ExecTreeNode *n = p.second;
    if (other_n->left) {
      ExecTreeNode *left = new ExecTreeNode(*other_n->left);
      addNode(left);
      n->left = left;
      left->parent = n;
    } else {
      n->left = nullptr;
    }
    if (other_n->right) {
      ExecTreeNode *right = new ExecTreeNode(*other_n->right);
      addNode(right);
      n->right = right;
      right->parent = n;
    } else {
      n->right = nullptr;
    }
  }
}

void ExecTree::addNode(ExecTreeNode *node) {
  nodes.push_back(node);
}

void ExecTree::extend(uint32_t stateID,
                      ref<Expr> condition,
                      uint32_t trueStateID,
                      uint32_t falseStateID,
                      uint32_t salt) {
  ExecTreeNode *left = new ExecTreeNode(falseStateID,
                                        Expr::createIsZero(condition),
                                        salt);
  ExecTreeNode *right = new ExecTreeNode(trueStateID,
                                         condition,
                                         salt);

  for (ExecTreeNode *node : nodes) {
    if (node->stateID == stateID && node->isLeaf()) {
      node->left = left;
      node->right = right;
      left->parent = node;
      right->parent = node;
      addNode(left);
      addNode(right);
      return;
    }
  }

  assert(false);
}

void ExecTree::computeHashes() {
  computeNodeHashes(root);
}

void ExecTree::computeNodeHashes(ExecTreeNode *n) {
  if (n->isLeaf()) {
    n->treeHash = n->getHash();
  } else {
    computeNodeHashes(n->left);
    computeNodeHashes(n->right);
    n->treeHash = n->getHash() + n->left->treeHash + n->right->treeHash;
  }
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

void ExecTree::dumpGML(llvm::raw_ostream &os, std::set<uint32_t> &ids) {
    os << "digraph G {\n";
    os << "\tsize=\"10,7.5\";\n";
    os << "\tratio=fill;\n";
    os << "\tdpi=600;\n";
    os << "\tcenter = \"true\";\n";
    os << "\tnode [style=\"filled\",width=1,height=1,fontname=\"Terminus\"]\n";
    os << "\tedge [arrowsize=.3]\n";

    std::vector<ExecTreeNode *> worklist;
    worklist.push_back(root);
    while (!worklist.empty()) {
        ExecTreeNode *n = worklist.back();
        worklist.pop_back();

        os << "\tn" << n << " [";
        if (n->e.isNull()) {
            os << "label=\"\"";
        } else {
            os << "label=\"" << n->getHash();
            os << "\",shape=square";
        }
        if (n->isLeaf() && ids.find(n->stateID) != ids.end()) {
            os << ",fillcolor=red";
        }
        os << "];\n";

        if (!n->isLeaf()) {
            os << "\tn" << n << " -> n" << n->left << ";\n";
            worklist.push_back(n->left);
            os << "\tn" << n << " -> n" << n->right << ";\n";
            worklist.push_back(n->right);
        }
    }
    os << "}\n";
}

void ExecTree::dumpGMLToFile(std::set<uint32_t> &ids, std::string &name) {
  static int mergeID = 0;
  char path[1000] = {0,};
  sprintf(path, "/tmp/exectree_%s_%u.dot", name.data(), mergeID++);
  std::error_code ec;
  raw_fd_ostream *f = new raw_fd_ostream(path, ec, sys::fs::F_None);
  if (f) {
    dumpGML(*f, ids);
    f->close();
  }
}

}
