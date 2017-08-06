# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "lanes",
    srcs = [
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "road_geometry.cc",
        "segment.cc",
        "spline_helpers.cc",
        "spline_lane.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "road_geometry.h",
        "segment.h",
        "spline_helpers.h",
        "spline_lane.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/math:geometric_transform",
        "//drake/math:saturate",
        "@ignition_math",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "ignition_rndf_test",
    deps = [
        "@ignition_rndf",
    ],
)

drake_cc_library(
    name = "ignition_types_compare",
    testonly = 1,
    srcs = ["test/ignition_types_compare.cc"],
    hdrs = ["test/ignition_types_compare.h"],
    deps = [
        "@gtest//:without_main",
        "@ignition_rndf",
    ],
)

drake_cc_googletest(
    name = "junction_test",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "road_geometry_test",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "segment_test",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "spline_lane_test",
    deps = [
        ":lanes",
        "//drake/automotive/maliput/api:maliput_types_compare",
    ],
)

drake_cc_googletest(
    name = "spline_helpers_test",
    deps = [
        ":ignition_types_compare",
        ":lanes",
    ],
)

add_lint_tests()
