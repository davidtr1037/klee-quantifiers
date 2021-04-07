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

Symbol &Word::operator[] (size_t n) {
  assert(n < symbols.size());
  return symbols[n];
}

size_t Word::size() const {
  return symbols.size();
}

void Word::append(Symbol &s) {
  symbols.push_back(s);
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

void Pattern::dump() const {
  errs() << "(" << prefix << ")";
  errs() << "(" << core << ")*";
  errs() << "(" << suffix << ")";
  errs() << "\n";
}

void PatternInstance::addSymbol(Symbol &s) {
  if (core.isEmpty()) {
    prefix.append(s);

    Word foundPrefix, foundCore;
    if (findRepitition(prefix, foundPrefix, foundCore)) {
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

bool PatternInstance::findRepitition(Word &input, Word &prefix, Word &core) {
  for (unsigned i = 0; i < input.size(); i++) {
    Word current_prefix, current_core;

    if (i > 0) {
      current_prefix.append(input[i - 1]);
    }

    bool hasPattern = true;
    size_t tailSize = input.size() - i;
    for (unsigned j = 0; j < tailSize / 2; j++) {
      if (input[i + j] != input[i + (tailSize / 2) + j]) {
        hasPattern = false;
        break;
      }
      current_core.append(input[i + j]);
    }

    /* if found a non-empty pattern */
    if (hasPattern && !current_core.isEmpty()) {
      prefix = current_prefix;
      core = current_core;
      return true;
    }
  }

  return false;
}

}
