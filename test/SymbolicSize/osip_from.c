// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out
// RUN: %klee --output-dir=%t.klee-out -libc=uclibc -validate-merge -use-loop-merge -loop-limit=none -use-optimized-merge=1 -allocate-sym-size -capacity=200 --search=dfs -optimize-ite-using-exec-tree -optimize-array-ite-using-exec-tree -use-merge-transformation=1 -use-join-transformation=1 -use-incremental-merging-search %t.bc 2>&1

#include <string.h>
#include "common.h"

#define MAX_SIZE (6)

int main(int argc, char *argv[]) {
    KLEE_MAKE_SYMBOLIC_STR_AND_LEN(s, n, MAX_SIZE);

    while (s[0] != '\0') {
        if (s[0] == ' ') {
            s++;
            continue;
        }

        if (s[0] >= 'a' && s[0] <= 'z') {
            s++;
            continue;
        }

        if (s[0] >= 'A' && s[0] <= 'Z') {
            s++;
            continue;
        }

        if (s[0] >= '0' && s[0] <= '9') {
            s++;
            continue;
        }

        if (s[0] == '-' || \
            s[0] == '.' || \
            s[0] == '!' || \
            s[0] == '%' || \
            s[0] == '*' || \
            s[0] == '_' || \
            s[0] == '+' || \
            s[0] == '`' || \
            s[0] == '\'' || \
            s[0] == '~') {
            s++;
            continue;
        }

        break;
    }

    return 0;
}
