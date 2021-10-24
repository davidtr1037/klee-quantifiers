// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc -validate-merge -use-loop-merge -loop-limit=none -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs -optimize-ite-using-exec-tree -optimize-array-ite-using-exec-tree -use-merge-transformation=1 -use-join-transformation=1 -use-incremental-merging-search %t.bc 2>&1

#include <string.h>
#include "common.h"

#define MAX_SIZE (10)

int main(int argc, char *argv[]) {
    KLEE_MAKE_SYMBOLIC_SIZE_NON_ZERO(n, size_t, MAX_SIZE);
    KLEE_MAKE_SYMBOLIC_STR(s1, n);
    KLEE_MAKE_SYMBOLIC_STR(s2, n);

    int r = strncasecmp(s1, s2, n - 1);

    return 0;
}
