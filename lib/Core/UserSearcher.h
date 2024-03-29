//===-- UserSearcher.h ------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_USERSEARCHER_H
#define KLEE_USERSEARCHER_H

#include "llvm/Support/CommandLine.h"

namespace klee {
  class Executor;
  class Searcher;

  extern llvm::cl::opt<bool> UseIncrementalMergingSearch;

  // XXX gross, should be on demand?
  bool userSearcherRequiresMD2U();

  void initializeSearchOptions();

  Searcher *constructUserSearcher(Executor &executor);
}

#endif /* KLEE_USERSEARCHER_H */
