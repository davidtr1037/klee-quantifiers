# State Merging with Quantifiers
This is an extension of KLEE which enables to perform state mering using quantified constraints.
It is based on the paper: *State Merging with Quantifiers in Symbolic Execution*,
which is available [here](https://doi.org/10.1145/3611643.3616287).

## Usage (Manual State Merging)
Consider the following example:
```
#include <stdlib.h>
#include <string.h>
#include <klee/klee.h>

int main(int argc, char *argv[]) {
  size_t length;
  klee_make_symbolic(&length, sizeof(length), "length");
  klee_assume(length >= 1);
  klee_assume(length <= 4);

  char *str = malloc(length);
  klee_make_symbolic(str, length, "str");
  str[length - 1] = 0;

  klee_open_qmerge();
  char *p = strchr(str, 'a');
  klee_close_qmerge();
  if (p) {
    klee_print_expr("p", p);
  }

  return 0;
}
```

To compile it, run the following command:
```
clang -g -c -emit-llvm -I <klee_src>/include <source_file> -o <bitcode_file>
```

Then, run the following command:
```
klee \
    -libc=uclibc \
    -solver-backend=z3 \
    -z3-smt-relevancy=1 \
    -search=dfs \
    -write-kqueries \
    -split-by-pattern=1 \
    -use-quantifiers=1 \
    -optimize-using-exec-tree=1 \
    -allocate-sym-size=1 \
    -capacity=10 \
    -optimize-using-quantifiers=1 \
    -use-small-model-solver=1 \
    <bitcode_file>
```

## Usage (Automatic State Merging)
In the previous example, state merging is applied manually.
Our implementation supports automatic application of state merging in size-dependent loops.
Consider the following example:
```
#include <stdlib.h>
#include <string.h>
#include <klee/klee.h>

int main(int argc, char *argv[]) {
  size_t length;
  klee_make_symbolic(&length, sizeof(length), "length");
  klee_assume(length >= 1);
  klee_assume(length <= 4);

  char *str = malloc(length);
  klee_make_symbolic(str, length, "str");
  str[length - 1] = 0;

  char *p = strchr(str, 'a');
  if (p) {
    klee_print_expr("p", p);
  }

  return 0;
}
```

To compile it, run the following command:
```
clang -g -c -emit-llvm -I <klee_src>/include <source_file> -o <bitcode_file>
```

Then, run the following command:
```
klee \
    -libc=uclibc \
    -solver-backend=z3 \
    -z3-smt-relevancy=1 \
    -search=dfs \
    -write-kqueries \
    -use-loop-merge=1 \
    -split-by-pattern=1 \
    -use-quantifiers=1 \
    -optimize-using-exec-tree=1 \
    -allocate-sym-size=1 \
    -capacity=10 \
    -use-incremental-merging-search=1 \
    -use-small-model-solver=1 \
    <bitcode_file>
```

Here, state merging will be applied in the `strchr` loop and will result in two merged symbolic states.
The resulting quantified path constraints are located in the output directory generated by KLEE:
```
<klee_out>/test000006.kquery
<klee_out>/test000007.kquery
```
