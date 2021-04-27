#ifndef KLEE_PATERN_H
#define KLEE_PATERN_H

#include <klee/Expr/Expr.h>

#include "llvm/Support/raw_ostream.h"

#include <stdint.h>
#include <vector>

namespace klee {

class Symbol {
public:

  Symbol(uint64_t hash) :
    hash(hash) {

  }

  bool operator==(const Symbol &other) const;

  bool operator!=(const Symbol &other) const;

  friend llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const Symbol &s);

  uint64_t hash;
};

class Word {
public:

  Word(std::vector<Symbol> &symbols) :
    symbols(symbols) {

  }

  Word() {

  }

  bool isEmpty() const;

  bool operator==(const Word &other) const;

  bool operator!=(const Word &other) const;

  Symbol &operator[] (size_t n);

  size_t size() const;

  void append(const Symbol &s);

  /* TODO: add operator+= */
  friend Word operator+(Word lhs, const Word &rhs);

  void clear();

  friend llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const Word &s);

  std::vector<Symbol> symbols;
};

class Pattern {
public:

  Pattern(const Word &prefix, const Word &core, const Word &suffix) :
    prefix(prefix), core(core), suffix(suffix) {

  }

  Pattern() {

  }

  bool hasCore() const;

  void dump() const;

  Word prefix;
  Word core;
  Word suffix;
};

/* TODO: rename? */
class PatternInstance : public Pattern {
public:

  PatternInstance() : count(0) {

  }

  PatternInstance(Word &prefix) : Pattern(prefix, Word(), Word()), count(0) {

  }

  void addSymbol(Symbol &s);

  bool isInstanceOf(Pattern &p, unsigned &repetitions);

  unsigned count;

private:

  bool findRepetition(Word &input, Word &prefix, Word &core);
};

}

#endif
