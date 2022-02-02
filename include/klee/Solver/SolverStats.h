//===-- SolverStats.h -------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_SOLVERSTATS_H
#define KLEE_SOLVERSTATS_H

#include "klee/Statistics/Statistic.h"

namespace klee {
namespace stats {

  extern Statistic cexCacheTime;
  extern Statistic queries;
  extern Statistic queriesInvalid;
  extern Statistic queriesValid;
  extern Statistic queryCacheHits;
  extern Statistic queryCacheMisses;
  extern Statistic queryCexCacheHits;
  extern Statistic queryCexCacheMisses;
  extern Statistic queryConstructs;
  extern Statistic queryCounterexamples;
  extern Statistic queryTime;
  extern Statistic auxilaryQueries;
  extern Statistic smallModelHits;
  extern Statistic smallModelMisses;
  extern Statistic smallModelUnsupported;
  extern Statistic smallModelTime;
  extern Statistic smallModelInitialQueryTime;
  extern Statistic smallModelResolveQueryTime;
  extern Statistic smallModelFallbackQueryTime;
  
#ifdef KLEE_ARRAY_DEBUG
  extern Statistic arrayHashTime;
#endif

}
}

#endif /* KLEE_SOLVERSTATS_H */
