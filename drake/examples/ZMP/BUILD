# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

drake_cc_library(
    name = "zmp_test_util",
    testonly = 1,
    srcs = ["zmp_test_util.cc"],
    hdrs = ["zmp_test_util.h"],
    deps = [
        "//drake/systems/controllers:zmp_planner",
    ],
)

drake_cc_binary(
    name = "zmp_example",
    testonly = 1,
    srcs = ["zmp_example.cc"],
    deps = [
        ":zmp_test_util",
        "//drake/lcm:lcm_call_matlab",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "zmp_planner_test",
    deps = [
        ":zmp_test_util",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
