//===-- KInstruction.cpp --------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Module/KInstruction.h"
#include <string>

using namespace llvm;
using namespace klee;

/***/

KInstruction::KInstruction() : loop(nullptr), isLoopEntry(false), isLoopExit(false) {

}

KInstruction::~KInstruction() {
  delete[] operands;
}

std::string KInstruction::getSourceLocation() const {
  if (!info->file.empty())
    return info->file + ":" + std::to_string(info->line) + " " +
           std::to_string(info->column);
  else return "[no debug info]";
}
