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
                           PTreeNode *ptreeNode,
                           std::uint32_t salt) :
  stateID(stateID),
  e(e),
  ptreeNode(ptreeNode),
  left(nullptr),
  right(nullptr),
  parent(nullptr),
  isJoined(false),
  salt(salt) {
  if (snapshot) {
    snapshots.push_back(snapshot);
  }
}

ExecTreeNode::~ExecTreeNode() {
  for (ExecutionState *snapshot : snapshots) {
    delete snapshot;
  }
}

/* TODO: pass a reference to ExecutionState? */
ExecTree::ExecTree(uint32_t stateID) {
  /* TODO: add snapshot/ptreeNode? */
  root = new ExecTreeNode(stateID,
                          ConstantExpr::create(1, Expr::Bool),
                          nullptr,
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

void ExecTree::setLeft(ExecTreeNode *parent,
                       ExecutionState &state,
                       ref<Expr> condition,
                       ExecutionState *snapshot,
                       std::uint32_t salt) {
  ExecTreeNode *node = new ExecTreeNode(state.getID(),
                                        condition,
                                        snapshot,
                                        state.ptreeNode,
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
                                        state.ptreeNode,
                                        salt);
  setRight(parent, node);
  addNode(node);
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
                                        falseState.ptreeNode,
                                        salt);
  ExecTreeNode *right = new ExecTreeNode(trueState.getID(),
                                         condition,
                                         trueSnapshot,
                                         trueState.ptreeNode,
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

void ExecTree::addSnapshot(ExecutionState &state,
                           ExecutionState *snapshot) {
  for (ExecTreeNode *node : nodes) {
    if (node->stateID == state.getID() && node->isLeaf()) {
      node->snapshots.push_back(snapshot);
      return;
    }
  }

  assert(0);
}

ExecTreeNode *ExecTree::getNearestAncestor(ExecTreeNode *n1,
                                           ExecTreeNode *n2) const {
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
                            std::vector<ExecTreeNode *> &reachable) const {
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

ref<Expr> ExecTree::getPC(ExecTreeNode *parent,
                          ExecTreeNode *child,
                          bool inclusive) const {
  std::list<ref<Expr>> conditions;
  ExecTreeNode *n = child;
  while (n && n != parent) {
    conditions.push_front(n->e);
    n = n->parent;
    assert(n);
  }
  if (inclusive) {
    conditions.push_front(parent->e);
  }

  ref<Expr> pc = ConstantExpr::create(1, Expr::Bool);
  for (ref<Expr> e : conditions) {
    pc = AndExpr::create(pc, e);
  }
  return pc;
}

void ExecTree::removeSubTree(ExecTreeNode *dst,
                             ExecTreeNode *stopAt) {
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
  while (current && current != stopAt) {
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
    if (sibling) {
      /* the parent can't be deleted */
      break;
    }

    current = next;
  }
}

bool ExecTree::join(ExecTreeNode *dst) {
  ExecTreeNode *current = dst;
  std::vector<ExecTreeNode *> toRemove;

  ExecTreeNode *parent = current->parent;
  while (parent && parent != root && !parent->isComplete()) {
    toRemove.push_back(parent);
    current = parent;
    parent = parent->parent;
  }

  if (current == dst) {
    /* nothing to do... */
    return false;
  }

  assert(parent && (parent->isComplete() || parent == root));
  if (parent->left == current) {
    parent->left = dst;
  }
  if (parent->right == current) {
    parent->right = dst;
  }

  dst->e = getPC(current, dst, true);
  dst->parent = parent;
  dst->isJoined = true;

  for (ExecTreeNode *n : toRemove) {
    removeNode(n);
  }

  return true;
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

/* TODO: indentation */
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
  sprintf(path, "/tmp/exectree_%04u_%s.dot", mergeID++, name.data());
  std::error_code ec;
  raw_fd_ostream f(path, ec, sys::fs::F_None);
  dumpGML(f, ids);
  f.close();
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

void ExecTree::setLeft(ExecTreeNode *parent, ExecTreeNode *node) {
  parent->left = node;
  node->parent = parent;
}

void ExecTree::setRight(ExecTreeNode *parent, ExecTreeNode *node) {
  parent->right = node;
  node->parent = parent;
}

}
