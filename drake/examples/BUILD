# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_binary")
load("//tools:lint.bzl", "add_lint_tests")

exports_files([
    "CPPLINT.cfg",
])

drake_cc_binary(
    name = "simple_continuous_time_system",
    srcs = ["simple_continuous_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:vector_system",
    ],
)

drake_cc_binary(
    name = "simple_discrete_time_system",
    srcs = ["simple_discrete_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:vector_system",
    ],
)

drake_cc_binary(
    name = "simple_mixed_continuous_and_discrete_time_system",
    srcs = ["simple_mixed_continuous_and_discrete_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:leaf_system",
    ],
)

add_lint_tests()
