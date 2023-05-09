#include "ExecTree.h"
#include "ExecutionState.h"

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/FileSystem.h"

#include <list>

using namespace std;
using namespace llvm;

namespace klee {

cl::opt<bool> DumpNodeCondition(
    "dump-node-condition",
    cl::init(false),
    cl::desc(""));

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

uint32_t shapeHashCallback(ExecTreeNode *node) {
  return node->getHash();
}

static uint32_t customHash(ref<Expr> e) {
  ref<EqExpr> eqExpr = dyn_cast<EqExpr>(e);
  if (!eqExpr.isNull()) {
    if (eqExpr->left->getWidth() == Expr::Bool && eqExpr->left->isFalse()) {
      /* (eq false e) */
      unsigned r = eqExpr->getKind() * Expr::MAGIC_HASH_CONSTANT;
      r <<= 1;
      r ^= eqExpr->left->shapeHash() * Expr::MAGIC_HASH_CONSTANT;
      r <<= 1;
      r ^= customHash(eqExpr->right) * Expr::MAGIC_HASH_CONSTANT;
      return r;
    } else {
      if (isa<ConstantExpr>(eqExpr->left) && isa<ReadExpr>(eqExpr->right)) {
        /* (eq c read(...)) */
        unsigned r = eqExpr->getKind() * Expr::MAGIC_HASH_CONSTANT;
        r <<= 1;
        /* don't ignore the hash of this constant,
         * i.e., use hash instead of shapeHash */
        r ^= eqExpr->left->hash() * Expr::MAGIC_HASH_CONSTANT;
        r <<= 1;
        r ^= eqExpr->right->shapeHash() * Expr::MAGIC_HASH_CONSTANT;
        return r;
      }
    }
  }

  return e->shapeHash();
}

uint32_t customHashCallback(ExecTreeNode *node) {
  return customHash(node->e);
}

/* TODO: pass a reference to ExecutionState? */
ExecTree::ExecTree(uint32_t stateID) {
  /* TODO: add snapshot/ptreeNode? */
  root = new ExecTreeNode(stateID,
                          ConstantExpr::create(1, Expr::Bool),
                          nullptr,
                          nullptr);
  addNode(root);
  setHashCallback(&shapeHashCallback);
}

ExecTree::~ExecTree() {
  clear();
}

void ExecTree::setHashCallback(HashCallback callback) {
  hashCallback = callback;
}

std::uint32_t ExecTree::getNodeHash(ExecTreeNode *node) const {
  assert(hashCallback != nullptr);
  return hashCallback(node);
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

bool ExecTree::extend(ExecutionState &current,
                      ExecutionState *trueState,
                      ExecutionState *falseState,
                      ref<Expr> condition,
                      std::uint32_t salt) {
  assert(trueState || falseState);
  ExecTreeNode *left = nullptr, *right = nullptr;

  if (falseState) {
    left = new ExecTreeNode(falseState->getID(),
                            Expr::createIsZero(condition),
                            nullptr,
                            falseState->ptreeNode,
                            salt);
  }

  if (trueState) {
    right = new ExecTreeNode(trueState->getID(),
                             condition,
                             nullptr,
                             trueState->ptreeNode,
                             salt);
  }

  for (ExecTreeNode *node : nodes) {
    if (node->stateID == current.getID() && node->isLeaf()) {
      if (left) {
        node->left = left;
        left->parent = node;
        addNode(left);
      }
      if (right) {
        node->right = right;
        right->parent = node;
        addNode(right);
      }

      /* TODO: check validity when merging nodes */
      return !left || !right || left->getHash() != right->getHash();
    }
  }

  assert(false);
  return false;
}

void ExecTree::addSnapshot(ExecutionState &state,
                           ExecutionState *snapshot) {
  for (ExecTreeNode *node : nodes) {
    if (node->stateID == state.getID() && node->isLeaf()) {
      node->snapshots.push_back(snapshot);
      nodesToMerge.insert(node);
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

/* TODO: rename */
bool ExecTree::isSinglePath(ExecTreeNode *node, ExecTreeNode *ancestor) {
  ExecTreeNode *current = node->parent;
  while (current && current != ancestor) {
    if (current->isComplete()) {
      return false;
    }
    current = current->parent;
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
            if (DumpNodeCondition) {
              os << "label=\"" << *n->e;
            } else {
              os << "label=\"" << getNodeHash(n);
            }
            os << "\",shape=square";
        }
        if (n->isLeaf() && ids.find(n->stateID) != ids.end()) {
            os << ",fillcolor=sandybrown";
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
  if (!node->snapshots.empty()) {
    nodesToMerge.insert(node);
  }
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
