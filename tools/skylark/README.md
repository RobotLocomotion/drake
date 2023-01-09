
This directory contains build system files related to the Bazel build system.
  https://bazel.build/

Specifically, files named `*.bzl` are Skylark extensions.
  https://bazel.build/versions/master/docs/skylark/concepts.html

This folder is intended to house skylark code used by Drake.  It contains both
generic functions such as `pathutils.bzl`, and widely-used but drake-specific
functions such as the `drake_{cc,java,py,...}.bzl` language-specific helpers.


# Common options

Several of the macros in Drake (e.g., drake_cc_googletest, drake_py_unittest,
etc.) provide for additional options beyond what is built-in to Bazel.  In
cases where those options are shared across several different rules, we
document the option here to avoid repetition.

**num_threads**

Can either be None, or else an integer.

When None or 1, adds `OMP_NUM_THREADS="1"` to `env` to disable OpenMP.

When N>1, adds `OMP_NUM_THREADS="N"` to `env` to allow for the desired level of
parallelism, and also adds "cpu:N" to `tags` to reserve sufficient CPUs for the
test. Those changes are a *necessary* but not *sufficient* condition to enable
actual CPU parallelism while running the test. In addition, the overall build
must also have OpenMP enabled, via --config=omp or --config=everything.

Note that setting N>1 will also be applied to any sub-processes that are
launched by your test program.  Ask for help on Slack if you need this flag to
work correctly in the presence of sub-processes.

(Aside: Besides OMP_NUM_THREADS, the function also sets several other
environment variables using alternative spellings of the same concept;
the overall effect should be the same.)
