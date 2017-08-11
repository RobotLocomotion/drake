# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "penetration_as_point_pair",
    srcs = [],
    hdrs = ["penetration_as_point_pair.h"],
    deps = [
        "//drake/common",
        "//drake/geometry:geometry_ids",
    ],
)

add_lint_tests()
