# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "continuity_constraint",
    srcs = ["continuity_constraint.cc"],
    hdrs = ["continuity_constraint.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "spline_generation",
    srcs = ["spline_generation.cc"],
    hdrs = ["spline_generation.h"],
    linkstatic = 1,
    deps = [
        ":spline_information",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

cc_library(
    name = "spline_information",
    srcs = ["spline_information.cc"],
    hdrs = ["spline_information.h"],
    linkstatic = 1,
    deps = [
        ":continuity_constraint",
        ":value_constraint",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

cc_library(
    name = "value_constraint",
    srcs = ["value_constraint.cc"],
    hdrs = ["value_constraint.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

# === test/ ===

cc_googletest(
    name = "spline_generation_test",
    deps = [
        ":spline_generation",
    ],
)
