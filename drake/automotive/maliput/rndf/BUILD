# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

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

drake_cc_googletest(
    name = "junction_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "spline_helpers_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "road_geometry_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "segment_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

cpplint()
