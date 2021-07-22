#include "ExecTree.h"
#include "ExecutionState.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"

#include <list>

using namespace std;
using namespace llvm;

namespace klee {

ExecTreeNode::ExecTreeNode(std::uint32_t stateID,
                           ref<Expr> e,
                           ExecutionState *snapshot,
                           std::uint32_t salt) :
  stateID(stateID),
  e(e),
  snapshot(snapshot),
  left(nullptr),
  right(nullptr),
  parent(nullptr),
  treeHash(0),
  salt(salt) {

}

ExecTreeNode::~ExecTreeNode() {
  if (snapshot) {
    delete snapshot;
  }
}

/* TODO: pass a reference to ExecutionState? */
ExecTree::ExecTree(uint32_t stateID) {
  /* TODO: add snapshot? */
  root = new ExecTreeNode(stateID,
                          ConstantExpr::create(1, Expr::Bool),
                          nullptr);
  addNode(root);
}

ExecTree::~ExecTree() {
  clear();
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
  nodes.insert(node);
}

void ExecTree::removeNode(ExecTreeNode *node) {
  auto i = nodes.find(node);
  assert(i != nodes.end());
  nodes.erase(i);
  delete node;
}

void ExecTree::setLeft(ExecTreeNode *parent,
                       ExecutionState &state,
                       ref<Expr> condition,
                       ExecutionState *snapshot,
                       std::uint32_t salt) {
  ExecTreeNode *node = new ExecTreeNode(state.getID(),
                                        condition,
                                        snapshot,
                                        salt);
  setLeft(parent, node);
  addNode(node);
}

void ExecTree::setRight(ExecTreeNode *parent,
                        ExecutionState &state,
                        ref<Expr> condition,
                        ExecutionState *snapshot,
                        std::uint32_t salt) {
  ExecTreeNode *node = new ExecTreeNode(state.getID(),
                                        condition,
                                        snapshot,
                                        salt);
  setRight(parent, node);
  addNode(node);
}

void ExecTree::setLeft(ExecTreeNode *parent, ExecTreeNode *node) {
  parent->left = node;
  node->parent = parent;
}

void ExecTree::setRight(ExecTreeNode *parent, ExecTreeNode *node) {
  parent->right = node;
  node->parent = parent;
}

void ExecTree::extend(ExecutionState &current,
                      ExecutionState &trueState,
                      ExecutionState *trueSnapshot,
                      ExecutionState &falseState,
                      ExecutionState *falseSnapshot,
                      ref<Expr> condition,
                      std::uint32_t salt) {
  ExecTreeNode *left = new ExecTreeNode(falseState.getID(),
                                        Expr::createIsZero(condition),
                                        falseSnapshot,
                                        salt);
  ExecTreeNode *right = new ExecTreeNode(trueState.getID(),
                                         condition,
                                         trueSnapshot,
                                         salt);

  for (ExecTreeNode *node : nodes) {
    if (node->stateID == current.getID() && node->isLeaf()) {
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

/* currently unused */
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

ExecTreeNode *ExecTree::getNearestAncestor(ExecTreeNode *n1, ExecTreeNode *n2) {
  std::vector<ExecTreeNode *> p1;
  std::set<ExecTreeNode *> p2;
  ExecTreeNode *current;

  current = n1;
  while (current) {
    p1.push_back(current);
    current = current->parent;
  }

  current = n2;
  while (current) {
    p2.insert(current);
    current = current->parent;
  }

  for (ExecTreeNode *n1 : p1) {
    if (p2.find(n1) != p2.end()) {
      return n1;
    }
  }

  return nullptr;
}

void ExecTree::getReachable(ExecTreeNode *src,
                            std::vector<ExecTreeNode *> &reachable) {
  std::list<ExecTreeNode *> worklist;
  worklist.push_back(src);

  while (!worklist.empty()) {
    ExecTreeNode *n = worklist.back();
    worklist.pop_back();

    reachable.push_back(n);

    if (n->left) {
      worklist.push_back(n->left);
    }
    if (n->right) {
      worklist.push_back(n->right);
    }
  }
}

/* from the child to the parent */
ref<Expr> ExecTree::getPC(ExecTreeNode *from, ExecTreeNode *to) {
  std::list<ref<Expr>> conditions;
  ExecTreeNode *n = from;
  while (n && n != to) {
    conditions.push_front(n->e);
    n = n->parent;
  }
  assert(n);

  ref<Expr> pc = ConstantExpr::create(1, Expr::Bool);
  for (ref<Expr> e : conditions) {
    pc = AndExpr::create(pc, e);
  }
  return pc;
}

/* TODO: check when the root is to be deleted (and assert) */
/* TODO: rename (removeSubTree) */
void ExecTree::removePathTo(ExecTreeNode *dst) {
  /* remove the reachable nodes */
  std::vector<ExecTreeNode *> reachable;
  /* TODO: don't compute twice */
  getReachable(dst, reachable);
  for (ExecTreeNode *n : reachable) {
    if (n != dst) {
      removeNode(n);
    }
  }

  ExecTreeNode *current = dst;
  while (current) {
    /* we don't want to delet the root */
    assert(current->parent);

    /* call before updating the parent */
    ExecTreeNode *sibling = current->getSibling();

    /* update the parent */
    if (current->parent->right == current) {
      current->parent->right = nullptr;
    }
    if (current->parent->left == current) {
      current->parent->left = nullptr;
    }

    ExecTreeNode *next = current->parent;
    removeNode(current);
    current = next;
    if (sibling) {
      /* the parent can't be deleted */
      break;
    }
  }
}

void ExecTree::clear() {
  for (ExecTreeNode *n : nodes) {
    delete n;
  }
  nodes.clear();
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
            if (n->left) {
              os << "\tn" << n << " -> n" << n->left << ";\n";
              worklist.push_back(n->left);
            }
            if (n->right) {
              os << "\tn" << n << " -> n" << n->right << ";\n";
              worklist.push_back(n->right);
            }
        }
    }
    os << "}\n";
}

void ExecTree::dumpGMLToFile(std::set<uint32_t> &ids, const std::string &name) {
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
