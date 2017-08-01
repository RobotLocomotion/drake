# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_binary(
    name = "zmp_example",
    testonly = 1,
    srcs = ["zmp_example.cc"],
    deps = [
        "//drake/common/proto:call_matlab",
        "//drake/systems/controllers:zmp_test_util",
    ],
)

# === test/ ===

add_lint_tests()
