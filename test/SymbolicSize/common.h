#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdlib.h>
#include <klee/klee.h>

#define KLEE_MAKE_SYMBOLIC(var, type) \
    type var; \
    klee_make_symbolic(&var, sizeof(var), #var);

#define KLEE_MAKE_SYMBOLIC_SIZE(var, type, max) \
    type var; \
    klee_make_symbolic(&var, sizeof(var), #var); \
    klee_assume(var >= 0); \
    klee_assume(var <= max);

#define KLEE_MAKE_SYMBOLIC_SIZE_NON_ZERO(var, type, max) \
    type var; \
    klee_make_symbolic(&var, sizeof(var), #var); \
    klee_assume(var >= 1); \
    klee_assume(var <= max);

#define KLEE_MAKE_SYMBOLIC_STR(str, max_size) \
    char *str = malloc(max_size); \
    klee_make_symbolic(str, max_size, #str); \
    str[max_size - 1] = 0;

#define KLEE_MAKE_SYMBOLIC_STR_AND_LEN(str, length, max_size) \
    KLEE_MAKE_SYMBOLIC_SIZE(length, size_t, max_size); \
    char *str = malloc(length + 1); \
    klee_make_symbolic(str, length + 1, #str); \
    str[length] = 0;

#endif
