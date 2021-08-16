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

  /* TODO: return a reference? */
  Symbol operator[] (size_t n) const;

  size_t size() const;

  void append(const Symbol &s);

  /* TODO: add operator+= */
  friend Word operator+(Word lhs, const Word &rhs);

  void clear();

  friend llvm::raw_ostream &operator<<(llvm::raw_ostream &os, const Word &s);

  Word reversed() const;

  static Word getCommonPrefix(const Word &w1, const Word &w2);

  static Word getCommonSuffix(const Word &w1, const Word &w2);

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

  Word getInstance(unsigned count) const;

  bool operator==(const Pattern &other) const;

  bool operator!=(const Pattern &other) const;

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

  PatternInstance(Pattern &p, unsigned count) :
    Pattern(p.prefix, p.core, p.suffix), count(count) {
    word = p.getInstance(count);
  }

  void addSymbol(const Symbol &s);

  PatternInstance reversed() const;

  Word word;

  unsigned count;

private:

  bool findRepetition(Word &input, Word &prefix, Word &core);
};

}

#endif
