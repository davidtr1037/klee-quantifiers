// RUN: %clang %s -emit-llvm %O0opt -c -o %t.bc
// RUN: rm -rf %t.klee-out

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include <klee/klee.h>

#define MAX_SIZE (1)

int f(unsigned char *p, size_t n) {
    int z = 0;
    for (unsigned j = 0; j < n; j++) {
        z += 2;
    }
    return z;
}

int main(int argc, char *argv[]) {
    size_t n;
    klee_make_symbolic(&n, sizeof(n), "n");
    klee_assume(n <= MAX_SIZE);

    unsigned char *p = malloc(n);
    for (unsigned i = 0; i < n; i++) {
        p[i] = 7;
        f(p, n);
    }

    return 0;
}
