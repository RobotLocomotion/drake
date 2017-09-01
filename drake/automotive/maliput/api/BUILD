# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "api",
    srcs = [
        "lane_data.cc",
        "road_geometry.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "lane_data.h",
        "road_geometry.h",
        "segment.h",
        "type_specific_identifier.h",
    ],
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

# === test/ ===
drake_cc_googletest(
    name = "lane_data_test",
    deps = [
        ":api",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "maliput_test",
    deps = [
        ":api",
    ],
)

drake_cc_googletest(
    name = "type_specific_identifier_test",
    deps = [
        ":api",
    ],
)

drake_cc_library(
    name = "maliput_types_compare",
    testonly = 1,
    srcs = ["test/maliput_types_compare.cc"],
    hdrs = ["test/maliput_types_compare.h"],
    deps = [
        ":api",
        "//drake/common",
        "//drake/math:geometric_transform",
        "@gtest//:without_main",
    ],
)

add_lint_tests()
