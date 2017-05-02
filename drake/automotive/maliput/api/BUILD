# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library", "drake_cc_googletest")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "api",
    srcs = [
        "road_geometry.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "lane_data.h",
        "road_geometry.h",
        "segment.h",
    ],
    deps = [
        "//drake/common",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "api_lane_data_test",
    size = "small",
    srcs = ["test/api_lane_data_test.cc"],
    deps = [
        ":api",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
