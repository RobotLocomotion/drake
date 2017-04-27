# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary")

exports_files([
    "CPPLINT.cfg",
])

drake_cc_binary(
    name = "simple_continuous_time_system",
    srcs = ["simple_continuous_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "simple_discrete_time_system",
    srcs = ["simple_discrete_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:leaf_system",
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

cpplint()
