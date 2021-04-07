#ifndef KLEE_PATERN_H
#define KLEE_PATERN_H

#include <klee/Expr/Expr.h>

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

  void dump() const;

  std::vector<Symbol> symbols;
};

class Pattern {
public:

  Pattern() { }

  Word prefix;
  Word core;
  Word suffix;
};

}

#endif
