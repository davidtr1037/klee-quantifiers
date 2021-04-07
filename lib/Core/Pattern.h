#ifndef KLEE_PATERN_H
#define KLEE_PATERN_H

#include <klee/Expr/Expr.h>

#include <stdint.h>
#include <vector>
#include <iostream>

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

  void append(Symbol &s);

  void clear();

  friend llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const Symbol &s);

  std::vector<Symbol> symbols;
};

class Pattern {
public:

  Pattern() { }

  void dump() const;

  Word prefix;
  Word core;
  Word suffix;
};

/* TODO: rename? */
class PatternInstance : public Pattern {
public:

  PatternInstance(unsigned count) : count(count) {

  }

  PatternInstance() {

  }

  void addSymbol(Symbol &s);

private:

  bool findRepitition(Word &input, Word &prefix, Word &core);

  unsigned count;
};

}

#endif
