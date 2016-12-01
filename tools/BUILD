# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

sh_binary(
    name = "valgrind",
    srcs = ["valgrind.sh"],
    data = ["valgrind.supp"],
)
