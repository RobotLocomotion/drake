
This directory contains build system files related to the Bazel build system.
  https://bazel.build/

Specifically, files named `*.bzl` are Skylark extensions.
  https://bazel.build/versions/master/docs/skylark/concepts.html

This folder is intended to house skylark code used by Drake.  It contains both
generic functions such as `pathutils.bzl`, and widely-used but drake-specific
functions such as the `drake_{cc,java,py,...}.bzl` language-specific helpers.


# Options shared across multiple macros

Several of the macros in Drake (e.g., drake_cc_googletest, drake_py_unittest,
etc.) provide for additional options beyond what is built-in to Bazel.  In
cases where those options are shared across several different rules, we
document the option here to avoid repetition.

**allow_network**

A list of components allowed to use the network, per IsNetworkingAllowed()
in drake/common/network_policy.h. When empty or not set, defaults to allowing
["meshcat"] but nothing else. You can set it to ["none"] to deny everything.

We allow meshcat by default because it's mostly harmless, a bajillion tests use
it by default, and it doesn't seem worthwhile to try to allow-list them all.

Note that this does not affect network sandboxing (i.e., Bazel's block-network
tag). Code outside of Drake purview can still access the network in tests (e.g.,
license servers for commercial solvers).

**display**

Can either be True or False (defaults to False).

When True, provides access to the Xorg DISPLAY environment variable so that
the test can use the X display.  When False, unsets DISPLAY to forbid access.

Note that the X display in Jenkins CI tends to crash frequently, so any tests
marked with `display = True` are a likely candidate for `flaky = True` so that
X crashes don't lead to false positives in CI.

**num_threads**

Can either be None, or else an integer.

When None or 1, adds `DRAKE_NUM_THREADS="1"` and `OMP_NUM_THREADS="1"` to `env`
to disable Drake's use of std::async and OpenMP.

When N>1, adds `DRAKE_NUM_THREADS="N"` and `OMP_NUM_THREADS="N"` to `env` to
allow for the desired level of parallelism, and also adds "cpu:N" to `tags` to
reserve sufficient CPUs for the test. Those changes are a *necessary* but might
not be *sufficient* condition to enable actual CPU parallelism while running the
test. In addition, the overall build must also have OpenMP enabled, via
--config=omp or --config=everything, if the code uses OpenMP for parallelism
(instead of std::async).

Note that setting N>1 will also be applied to any sub-processes that are
launched by your test program.  Ask for help on Slack if you need this flag to
work correctly in the presence of sub-processes.

(Aside: Besides the two named environment variables, the function also sets
several other environment variables using alternative spellings of the same
concept; the overall effect should be the same.)
