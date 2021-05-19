//===-- CachingSolver.cpp - Caching expression solver ---------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//


#include "klee/Solver/Solver.h"

#include "klee/Expr/Constraints.h"
#include "klee/Expr/Expr.h"
#include "klee/Solver/IncompleteSolver.h"
#include "klee/Solver/SolverImpl.h"
#include "klee/Solver/SolverStats.h"

#include "llvm/Support/CommandLine.h"

#include <unordered_map>

using namespace llvm;
using namespace klee;

namespace {

cl::opt<bool> UseIsoCache("use-iso-cache",
                          cl::init(false),
                          cl::desc(""));

}

class CachingSolver : public SolverImpl {
private:
  ref<Expr> canonicalizeQuery(ref<Expr> originalQuery,
                              bool &negationUsed);

  void cacheInsert(const Query& query,
                   IncompleteSolver::PartialValidity result);

  bool cacheLookup(const Query& query,
                   IncompleteSolver::PartialValidity &result);
  
  struct CacheEntry {
    CacheEntry(const ConstraintSet &c, ref<Expr> q)
        : constraints(c), query(q) {}

    CacheEntry(const CacheEntry &ce)
      : constraints(ce.constraints), query(ce.query) {}

    ConstraintSet constraints;
    ref<Expr> query;

    bool operator==(const CacheEntry &b) const {
      return constraints==b.constraints && *query.get()==*b.query.get();
    }
  };

  struct CacheEntryHash {
    unsigned operator()(const CacheEntry &ce) const {
      unsigned result = ce.query->hash();

      for (auto const &constraint : ce.constraints) {
        result ^= constraint->hash();
      }

      return result;
    }
  };

  struct IsoCacheEntry {
    IsoCacheEntry(const ConstraintSet &c, ref<Expr> q)
        : constraints(c), query(q) {}

    IsoCacheEntry(const CacheEntry &ce)
      : constraints(ce.constraints), query(ce.query) {}

    ConstraintSet constraints;
    ref<Expr> query;

    bool operator==(const IsoCacheEntry &other) const {
      if (constraints.size() != other.constraints.size()) {
        return false;
      }

      /* we must preserve the same mapping for all constraints (pc and expr) */
      ArrayMapping map;
      if (!query->isIsomorphic(*other.query, map)) {
        return false;
      }

      auto iter1 = constraints.begin();
      auto iter2 = other.constraints.begin();
      while (iter1 != constraints.end()) {
        ref<Expr> e1 = *iter1;
        ref<Expr> e2 = *iter2;
        if (!e1->isIsomorphic(*e2, map)) {
          return false;
        }
        iter1++; iter2++;
      }
      return true;
    }
  };

  struct IsoCacheEntryHash {
    unsigned operator()(const IsoCacheEntry &ce) const {
      unsigned result = ce.query->isoHash();
      for (ref<Expr> e : ce.constraints) {
        result ^= e->isoHash();
      }
      return result;
    }
  };

  typedef std::unordered_map<CacheEntry, IncompleteSolver::PartialValidity, CacheEntryHash> cache_map;
  typedef std::unordered_map<IsoCacheEntry, IncompleteSolver::PartialValidity, IsoCacheEntryHash> IsoCacheMap;

  Solver *solver;
  cache_map cache;
  IsoCacheMap isoCache;

public:
  CachingSolver(Solver *s) : solver(s) {}
  ~CachingSolver() { cache.clear(); isoCache.clear(); delete solver; }

  bool computeValidity(const Query&, Solver::Validity &result);
  bool computeTruth(const Query&, bool &isValid);
  bool computeValue(const Query& query, ref<Expr> &result) {
    ++stats::queryCacheMisses;
    return solver->impl->computeValue(query, result);
  }
  bool computeInitialValues(const Query& query,
                            const std::vector<const Array*> &objects,
                            std::vector< std::vector<unsigned char> > &values,
                            bool &hasSolution) {
    ++stats::queryCacheMisses;
    return solver->impl->computeInitialValues(query, objects, values, 
                                              hasSolution);
  }
  SolverRunStatus getOperationStatusCode();
  char *getConstraintLog(const Query&);
  void setCoreSolverTimeout(time::Span timeout);
};

/** @returns the canonical version of the given query.  The reference
    negationUsed is set to true if the original query was negated in
    the canonicalization process. */
ref<Expr> CachingSolver::canonicalizeQuery(ref<Expr> originalQuery,
                                           bool &negationUsed) {
  ref<Expr> negatedQuery = Expr::createIsZero(originalQuery);

  // select the "smaller" query to the be canonical representation
  if (originalQuery.compare(negatedQuery) < 0) {
    negationUsed = false;
    return originalQuery;
  } else {
    negationUsed = true;
    return negatedQuery;
  }
}

/** @returns true on a cache hit, false of a cache miss.  Reference
    value result only valid on a cache hit. */
bool CachingSolver::cacheLookup(const Query& query,
                                IncompleteSolver::PartialValidity &result) {
  bool negationUsed;
  ref<Expr> canonicalQuery = canonicalizeQuery(query.expr, negationUsed);

  if (UseIsoCache) {
    IsoCacheEntry ce(query.constraints, canonicalQuery);
    auto it = isoCache.find(ce);
    if (it != isoCache.end()) {
      result = (negationUsed ? IncompleteSolver::negatePartialValidity(it->second) : it->second);
      return true;
    }
  } else {
    CacheEntry ce(query.constraints, canonicalQuery);
    cache_map::iterator it = cache.find(ce);

    if (it != cache.end()) {
      result = (negationUsed ?
                IncompleteSolver::negatePartialValidity(it->second) :
                it->second);
      return true;
    }
  }
  
  return false;
}

/// Inserts the given query, result pair into the cache.
void CachingSolver::cacheInsert(const Query& query,
                                IncompleteSolver::PartialValidity result) {
  bool negationUsed;
  ref<Expr> canonicalQuery = canonicalizeQuery(query.expr, negationUsed);

  if (UseIsoCache) {
    IsoCacheEntry ce(query.constraints, canonicalQuery);
    IncompleteSolver::PartialValidity cachedResult =
      (negationUsed ? IncompleteSolver::negatePartialValidity(result) : result);

    isoCache.insert(std::make_pair(ce, cachedResult));
  } else {
    CacheEntry ce(query.constraints, canonicalQuery);
    IncompleteSolver::PartialValidity cachedResult =
      (negationUsed ? IncompleteSolver::negatePartialValidity(result) : result);

    cache.insert(std::make_pair(ce, cachedResult));
  }
}

bool CachingSolver::computeValidity(const Query& query,
                                    Solver::Validity &result) {
  IncompleteSolver::PartialValidity cachedResult;
  bool tmp, cacheHit = cacheLookup(query, cachedResult);
  
  if (cacheHit) {
    switch(cachedResult) {
    case IncompleteSolver::MustBeTrue:   
      result = Solver::True;
      ++stats::queryCacheHits;
      return true;
    case IncompleteSolver::MustBeFalse:  
      result = Solver::False;
      ++stats::queryCacheHits;
      return true;
    case IncompleteSolver::TrueOrFalse:  
      result = Solver::Unknown;
      ++stats::queryCacheHits;
      return true;
    case IncompleteSolver::MayBeTrue: {
      ++stats::queryCacheMisses;
      if (!solver->impl->computeTruth(query, tmp))
        return false;
      if (tmp) {
        cacheInsert(query, IncompleteSolver::MustBeTrue);
        result = Solver::True;
        return true;
      } else {
        cacheInsert(query, IncompleteSolver::TrueOrFalse);
        result = Solver::Unknown;
        return true;
      }
    }
    case IncompleteSolver::MayBeFalse: {
      ++stats::queryCacheMisses;
      if (!solver->impl->computeTruth(query.negateExpr(), tmp))
        return false;
      if (tmp) {
        cacheInsert(query, IncompleteSolver::MustBeFalse);
        result = Solver::False;
        return true;
      } else {
        cacheInsert(query, IncompleteSolver::TrueOrFalse);
        result = Solver::Unknown;
        return true;
      }
    }
    default: assert(0 && "unreachable");
    }
  }

  ++stats::queryCacheMisses;
  
  if (!solver->impl->computeValidity(query, result))
    return false;

  switch (result) {
  case Solver::True: 
    cachedResult = IncompleteSolver::MustBeTrue; break;
  case Solver::False: 
    cachedResult = IncompleteSolver::MustBeFalse; break;
  default: 
    cachedResult = IncompleteSolver::TrueOrFalse; break;
  }
  
  cacheInsert(query, cachedResult);
  return true;
}

bool CachingSolver::computeTruth(const Query& query,
                                 bool &isValid) {
  IncompleteSolver::PartialValidity cachedResult;
  bool cacheHit = cacheLookup(query, cachedResult);

  // a cached result of MayBeTrue forces us to check whether
  // a False assignment exists.
  if (cacheHit && cachedResult != IncompleteSolver::MayBeTrue) {
    ++stats::queryCacheHits;
    isValid = (cachedResult == IncompleteSolver::MustBeTrue);
    return true;
  }

  ++stats::queryCacheMisses;
  
  // cache miss: query solver
  if (!solver->impl->computeTruth(query, isValid))
    return false;

  if (isValid) {
    cachedResult = IncompleteSolver::MustBeTrue;
  } else if (cacheHit) {
    // We know a true assignment exists, and query isn't valid, so
    // must be TrueOrFalse.
    assert(cachedResult == IncompleteSolver::MayBeTrue);
    cachedResult = IncompleteSolver::TrueOrFalse;
  } else {
    cachedResult = IncompleteSolver::MayBeFalse;
  }
  
  cacheInsert(query, cachedResult);
  return true;
}

SolverImpl::SolverRunStatus CachingSolver::getOperationStatusCode() {
  return solver->impl->getOperationStatusCode();
}

char *CachingSolver::getConstraintLog(const Query& query) {
  return solver->impl->getConstraintLog(query);
}

void CachingSolver::setCoreSolverTimeout(time::Span timeout) {
  solver->impl->setCoreSolverTimeout(timeout);
}

///

Solver *klee::createCachingSolver(Solver *_solver) {
  return new Solver(new CachingSolver(_solver));
}
