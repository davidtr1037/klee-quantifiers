#include "Pattern.h"

#include "llvm/Support/raw_ostream.h"

using namespace std;
using namespace llvm;

namespace klee {

bool Symbol::operator==(const Symbol &other) const {
  return hash == other.hash;
}

bool Symbol::operator!=(const Symbol &other) const {
  return !operator==(other);
}

raw_ostream &operator<<(raw_ostream &os, const Symbol &s) {
  os << "[" << s.hash << "]";
  return os;
}

bool Word::isEmpty() const {
  return symbols.empty();
}

bool Word::operator==(const Word &other) const {
  return symbols == other.symbols;
}

bool Word::operator!=(const Word &other) const {
  return !operator==(other);
}

Symbol Word::operator[] (size_t n) const {
  assert(n < symbols.size());
  return symbols[n];
}

size_t Word::size() const {
  return symbols.size();
}

void Word::append(const Symbol &s) {
  symbols.push_back(s);
}

Word operator+(Word lhs, const Word &rhs) {
  Word sum = lhs;
  for (const Symbol &s : rhs.symbols) {
    sum.append(s);
  }
  return sum;
}

void Word::clear() {
  symbols.clear();
}

raw_ostream &operator<<(raw_ostream &os, const Word &w) {
  for (const Symbol &s : w.symbols) {
    os << s;
  }
  return os;
}

Word Word::reversed() const {
  Word w;
  size_t n = size();
  for (unsigned i = 0; i < n; i++) {
    w.append(symbols[n - i - 1]);
  }
  return w;
}

bool Pattern::hasCore() const {
  return !core.isEmpty();
}

Word Pattern::getInstance(unsigned count) const {
  Word w;
  w = w + prefix;
  for (unsigned i = 0; i < count; i++) {
    w = w + core;
  }
  w = w + suffix;
  return w;
}

bool Pattern::operator==(const Pattern &other) const {
  return prefix == other.prefix && \
         core == other.core && \
         suffix == other.suffix;
}

bool Pattern::operator!=(const Pattern &other) const {
  return !operator==(other);
}

void Pattern::dump() const {
  errs() << "(" << prefix << ")";
  errs() << "(" << core << ")*";
  errs() << "(" << suffix << ")";
  errs() << "\n";
}

void PatternInstance::addSymbol(const Symbol &s) {
  word.append(s);
  if (core.isEmpty()) {
    prefix.append(s);

    Word foundPrefix, foundCore;
    if (findRepetition(prefix, foundPrefix, foundCore)) {
      prefix = foundPrefix;
      core = foundCore;
      count = 2;
    }
  } else {
    suffix.append(s);
    if (suffix == core) {
      count++;
      suffix.clear();
    }
  }
}

/* TODO: remove */
bool PatternInstance::isInstanceOf(const Pattern &p, unsigned &repetitions) {
  if (!p.hasCore()) {
    return false;
  }

  if (count == 0) {
    /* if the instance has no core, then everything is in the prefix */
    assert(core.isEmpty());
    assert(suffix.isEmpty());

    /* try zero or one repetitions */
    for (unsigned i = 0; i < 2; i++) {
      /* generate */
      Word w = p.prefix;
      for (unsigned j = 0; j < i; j++) {
        w = w + p.core;
      }
      w = w + p.suffix;

      if (prefix == w) {
        repetitions = i;
        return true;
      }
    }
    return false;
  } else {
    if (prefix == p.prefix && core == p.core && suffix == p.suffix) {
      repetitions = count;
      return true;
    } else {
      return false;
    }
  }
}

PatternInstance PatternInstance::reversed() const {
  if (core.isEmpty()) {
    assert(suffix.isEmpty());
    Pattern p(prefix.reversed(), Word(), Word());
    return PatternInstance(p, count);
  } else {
    Pattern p(suffix.reversed(), core.reversed(), prefix.reversed());
    return PatternInstance(p, count);
  }
}

bool PatternInstance::findRepetition(Word &input,
                                     Word &out_prefix,
                                     Word &out_core) {
  for (unsigned i = 0; i < input.size(); i++) {
    Word current_prefix, current_core;
    bool hasPattern = true;
    size_t tailSize = input.size() - i;

    /* must be even */
    if (tailSize % 2 != 0) {
      continue;
    }

    for (unsigned j = 0; j < i; j++) {
      current_prefix.append(input[j]);
    }

    for (unsigned j = 0; j < tailSize / 2; j++) {
      if (input[i + j] != input[i + (tailSize / 2) + j]) {
        hasPattern = false;
        break;
      }
      current_core.append(input[i + j]);
    }

    /* if found a non-empty pattern */
    if (hasPattern && !current_core.isEmpty()) {
      out_prefix = current_prefix;
      out_core = current_core;
      return true;
    }
  }

  return false;
}

}
