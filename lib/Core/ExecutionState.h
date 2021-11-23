//===-- ExecutionState.h ----------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_EXECUTIONSTATE_H
#define KLEE_EXECUTIONSTATE_H

#include "AddressSpace.h"
#include "MergeHandler.h"
#include "LoopHandler.h"
#include "PatternExtraction.h"

#include "klee/ADT/TreeStream.h"
#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Expr/ExprVisitor.h"
#include "klee/Expr/ExprRename.h"
#include "klee/Module/KInstIterator.h"
#include "klee/Module/KLoop.h"
#include "klee/Solver/Solver.h"
#include "klee/System/Time.h"

#include <llvm/Analysis/LoopInfo.h>

#include <map>
#include <set>
#include <vector>
#include <unordered_map>

namespace klee {
class Array;
class CallPathNode;
struct Cell;
struct KFunction;
struct KInstruction;
class MemoryObject;
class PTreeNode;
struct InstructionInfo;

extern llvm::cl::opt<bool> OptimizeITEUsingExecTree;
extern llvm::cl::opt<bool> OptimizeArrayITEUsingExecTree;
extern llvm::cl::opt<bool> OptimizeArrayValuesByTracking;
extern llvm::cl::opt<bool> OptimizeUsingQuantifiers;
extern llvm::cl::opt<bool> CreateSnapshots;
extern llvm::cl::opt<bool> RewriteExpr;

llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const MemoryMap &mm);

struct StackFrame {
  KInstIterator caller;
  KFunction *kf;
  CallPathNode *callPathNode;

  std::vector<const MemoryObject *> allocas;
  Cell *locals;

  /// Minimum distance to an uncovered instruction once the function
  /// returns. This is not a good place for this but is used to
  /// quickly compute the context sensitive minimum distance to an
  /// uncovered instruction. This value is updated by the StatsTracker
  /// periodically.
  unsigned minDistToUncoveredOnReturn;

  // For vararg functions: arguments not passed via parameter are
  // stored (packed tightly) in a local (alloca) memory object. This
  // is set up to match the way the front-end generates vaarg code (it
  // does not pass vaarg through as expected). VACopy is lowered inside
  // of intrinsic lowering.
  MemoryObject *varargs;

  /* TODO: add docs */
  bool isExecutingLoop;
  /* TODO: use pointer? */
  KLoop loop;

  StackFrame(KInstIterator caller,
             KFunction *kf);
  StackFrame(KInstIterator caller,
             KFunction *kf,
             bool isExecutingLoop,
             KLoop loop);
  StackFrame(const StackFrame &s);
  ~StackFrame();
};

/// @brief ExecutionState representing a path under exploration
class ExecutionState {
  typedef std::vector<ref<Expr>> ExprSet;

  struct ExprKeyHash {
    unsigned operator()(const ref<Expr> &e) const {
      return e->hash();
    }
  };

  struct ExprKeyEquality {
    bool operator()(const ref<Expr> &e1, const ref<Expr> &e2) const {
      return e1->compare(*e2) == 0;
    }
  };

  struct MergedConstraint {
    ref<Expr> constraint;
    /* true iff all the reachable nodes belong to the same merging group */
    bool isFull;

    MergedConstraint(ref<Expr> constraint, bool isFull) :
      constraint(constraint),
      isFull(isFull) {

    }
  };


  struct MergedValue {
    ref<Expr> value;
    /* TODO: rename? */
    bool isComplete;
    /* TODO: rename? */
    ref<Expr> guard;

    MergedValue(ref<Expr> value, bool isComplete, ref<Expr> guard) :
      value(value),
      isComplete(isComplete),
      guard(guard) {

    }
  };

#ifdef KLEE_UNITTEST
public:
#else
private:
#endif
  // copy ctor
  ExecutionState(const ExecutionState &state, bool isSnapshot = false);

public:
  using stack_ty = std::vector<StackFrame>;

  // Execution - Control Flow specific

  /// @brief Pointer to instruction to be executed after the current
  /// instruction
  KInstIterator pc;

  /// @brief Pointer to instruction which is currently executed
  KInstIterator prevPC;

  /// @brief Stack representing the current instruction stream
  stack_ty stack;

  /// @brief Remember from which Basic Block control flow arrived
  /// (i.e. to select the right phi values)
  std::uint32_t incomingBBIndex;

  // Overall state of the state - Data specific

  /// @brief Exploration depth, i.e., number of times KLEE branched for this state
  std::uint32_t depth;

  /// @brief Address space used by this state (e.g. Global and Heap)
  AddressSpace addressSpace;

  /// @brief Constraints collected so far
  ConstraintSet constraints;

  /* TODO: add docs */
  ConstraintSet rewrittenConstraints;

  /// Statistics and information

  /// @brief Metadata utilized and collected by solvers for this state
  mutable SolverQueryMetaData queryMetaData;

  /// @brief History of complete path: represents branches taken to
  /// reach/create this state (both concrete and symbolic)
  TreeOStream pathOS;

  /// @brief History of symbolic path: represents symbolic branches
  /// taken to reach/create this state
  TreeOStream symPathOS;

  /// @brief Set containing which lines in which files are covered by this state
  std::map<const std::string *, std::set<std::uint32_t>> coveredLines;

  /// @brief Pointer to the process tree of the current state
  /// Copies of ExecutionState should not copy ptreeNode
  PTreeNode *ptreeNode = nullptr;

  /// @brief Ordered list of symbolics: used to generate test cases.
  //
  // FIXME: Move to a shared list structure (not critical).
  std::vector<std::pair<ref<const MemoryObject>, const Array *>> symbolics;

  /// @brief Set of used array names for this state.  Used to avoid collisions.
  std::set<std::string> arrayNames;

  /// @brief The objects handling the klee_open_merge calls this state ran through
  std::vector<ref<MergeHandler>> openMergeStack;

  /// @brief The numbers of times this state has run through Executor::stepInstruction
  std::uint64_t steppedInstructions;

  /// @brief Counts how many instructions were executed since the last new
  /// instruction was covered.
  std::uint32_t instsSinceCovNew;

  /// @brief the global state counter
  static std::uint32_t nextID;

  /// @brief the state id
  std::uint32_t id {0};

  /// @brief Whether a new instruction was covered in this state
  bool coveredNew;

  /// @brief Disables forking for this state. Set by user code
  bool forkDisabled;

  std::map<std::string, ExprSet> taintedExprs;

  ref<LoopHandler> loopHandler;

  ConstraintSet suffixConstraints;

  bool isSnapshot;

  /* TODO: add docs */
  bool hasPendingSnapshot;

  /* TODO: add docs */
  std::uint32_t localMergeID;

  /* translation map for renaming  */
  ArrayMap renamingMap;

public:
  #ifdef KLEE_UNITTEST
  // provide this function only in the context of unittests
  ExecutionState(){}
  #endif
  // only to create the initial state
  explicit ExecutionState(KFunction *kf);
  // no copy assignment, use copy constructor
  ExecutionState &operator=(const ExecutionState &) = delete;
  // no move ctor
  ExecutionState(ExecutionState &&) noexcept = delete;
  // no move assignment
  ExecutionState& operator=(ExecutionState &&) noexcept = delete;
  // dtor
  ~ExecutionState();

  ExecutionState *branch(bool isSnapshot = false);

  void pushFrame(KInstIterator caller, KFunction *kf);
  void popFrame();

  void addSymbolic(const MemoryObject *mo, const Array *array);

  void addConstraint(ref<Expr> e, bool atMerge = false);

  void addSuffixConstraint(ref<Expr> e);

  void clearSuffixConstraints();

  bool merge(const ExecutionState &b);
  void dumpStack(llvm::raw_ostream &out) const;

  std::uint32_t getID() const { return id; };
  void setID() { id = nextID++; };

  void addTaintedExpr(std::string name, ref<Expr> offset);

  bool hasTaintedExpr(std::string name, ref<Expr> offset);

  bool isTaintedExpr(ref<Expr> e);

  static ExecutionState *mergeStates(std::vector<ExecutionState *> &states);

  static ExecutionState *mergeStatesOptimized(std::vector<ExecutionState *> &states,
                                              bool isComplete,
                                              bool usePattern,
                                              std::vector<PatternMatch> &matches,
                                              LoopHandler *loopHandler);

  static bool canMerge(std::vector<ExecutionState *> &states,
                       std::set<const MemoryObject*> &mutated);

  /* TODO: too many parameters */
  static void mergeLocalVars(ExecutionState *merged,
                             std::vector<ExecutionState *> &states,
                             std::vector<ref<Expr>> &suffixes,
                             LoopHandler *loopHandler,
                             bool isEncodedWithABV,
                             std::vector<PatternMatch> &matches);

  /* TODO: too many parameters */
  static void mergeHeap(ExecutionState *merged,
                        std::vector<ExecutionState *> &states,
                        std::vector<ref<Expr>> &suffixes,
                        std::set<const MemoryObject*> &mutated,
                        LoopHandler *loopHandler,
                        bool isEncodedWithABV,
                        std::vector<PatternMatch> &matches);

  static ref<Expr> mergeValues(std::vector<ref<Expr>> &suffixes,
                               std::vector<ref<Expr>> &values);

  static ref<Expr> mergeValuesUsingExecTree(State2Value &valuesMap,
                                            LoopHandler *loopHandler);

  static MergedValue mergeValuesFromNode(ExecTreeNode *n,
                                         State2Value &valuesMap);

  static ref<Expr> mergeValuesUsingPattern(State2Value &valuesMap,
                                           LoopHandler *loopHandler,
                                           PatternMatch &pm,
                                           std::uint32_t mergeID);

  static bool areEquiv(TimingSolver *solver,
                       const ExecutionState *sa,
                       const ExecutionState *sb,
                       bool checkPCEquivalence);

  static bool shouldOptimizeArrayValues();

  void optimizeArrayValues(std::set<const MemoryObject*> mutated,
                           TimingSolver *solver);

  ref<Expr> simplifyArrayElement(const MemoryObject *mo,
                                 uint64_t offset,
                                 ref<Expr> v,
                                 TimingSolver *solver);

  static ref<Expr> mergeConstraints(std::vector<ExecutionState *> &states);

  static ref<Expr> mergeConstraintsWithExecTree(LoopHandler *loopHandler,
                                                std::vector<ExecutionState *> &states);

  static MergedConstraint mergeConstraintsFromNode(ExecTreeNode *n,
                                                   std::set<uint32_t> &ids);

  bool isValidOffset(TimingSolver *solver,
                     const MemoryObject *mo,
                     uint64_t offset);

  static bool isLiveRegAt(const LivenessAnalysis::Result &result,
                          KFunction *kf,
                          KInstruction *kinst,
                          unsigned reg);

  static bool compareStack(ExecutionState &s1,
                           ExecutionState &s2);

  static bool compareHeap(ExecutionState &s1,
                          ExecutionState &s2,
                          std::set<const MemoryObject *> &mutated);

  std::uint32_t getMergeID() const;

  void incMergeID();

  void addAuxArray();

  void mapAuxArrays();

  void rewriteConstraints();

private:
  static std::uint32_t mergeID;
};

struct ExecutionStateIDCompare {
  bool operator()(const ExecutionState *a, const ExecutionState *b) const {
    return a->getID() < b->getID();
  }
};

class TaintVisitor : public ExprVisitor {
protected:
  Action visitRead(const ReadExpr &e);

public:
  TaintVisitor(ExecutionState &state) : state(state), isTainted(false) {}
  ExecutionState &state;
  bool isTainted;
};

struct CodeLocation {

  CodeLocation(std::string file, unsigned line, std::string function) :
    file(file), line(line), function(function) {

  }

  bool operator==(const CodeLocation &other) const {
    return file == other.file && line == other.line &&  function == other.function;
  }

  void dump() const {
    llvm::errs() << file << ":" << line << " (" << function << ")" << "\n";
  }

  std::string file;
  unsigned line;
  std::string function;
};

class ExecutionContext {

public:

  ExecutionContext(ExecutionState &state, bool usePrevPC = true);

  bool operator==(const ExecutionContext &other) const {
    return trace == other.trace;
  }

  void dump() const;

  std::vector<CodeLocation> trace;
};

struct ExecutionContextHash {

  unsigned operator() (const ExecutionContext &context) const {
    unsigned result = 0;
    for (const CodeLocation &location : context.trace) {
      unsigned h1 = std::hash<std::string>()(location.file);
      unsigned h2 = std::hash<unsigned>()(location.line);
      unsigned h3 = std::hash<std::string>()(location.function);
      result ^= h1 ^ h2 ^ h3;
    }
    return result;
  }

};

}

#endif /* KLEE_EXECUTIONSTATE_H */
