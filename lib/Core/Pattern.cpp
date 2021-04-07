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

bool Word::isEmpty() const {
  return !symbols.empty();
}

bool Word::operator==(const Word &other) const {
  return symbols == other.symbols;
}

bool Word::operator!=(const Word &other) const {
  return !operator==(other);
}

/* TODO: pass stream */
void Word::dump() const {
  for (const Symbol &s : symbols) {
    errs() << "[" << s.hash << "]";
  }
  errs() << "\n";
}

}
