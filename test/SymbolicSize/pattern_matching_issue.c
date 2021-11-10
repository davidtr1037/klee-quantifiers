// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc -validate-merge -use-loop-merge -loop-limit=none -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs -optimize-ite-using-exec-tree -optimize-array-ite-using-exec-tree -use-merge-transformation=1 -use-join-transformation=1 %t.bc 2>&1

/* TODO: add test after handling the pattern matching issue */

#include <string.h>
#include "common.h"

#define MAX_SIZE (4)

int skip(char *s) {
    int i = 0;
    char *p = s;
    while ('a' == *p) {
        p++;
        i++;
    }
    return i;
}

int main(int argc, char *argv[]) {
    KLEE_MAKE_SYMBOLIC_STR_AND_LEN(s, n, MAX_SIZE);

    int k = skip(s);
    if (n >= 1 && s[0] != 'a') {
        while (s[k] != 0) {
            k++;
        }
    }

    return 0;
}
