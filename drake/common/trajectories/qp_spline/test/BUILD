# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    ":random_piecewise_polynomial",
    "//drake/common:eigen_matrix_compare",
    "@gtest//:main",
]

cc_test(
    name = "spline_generation_test",
    size = "small",
    srcs = ["spline_generation_test.cc"],
    deps = [
        "//drake/common/trajectories/qp_spline:spline_generation",
        "@gtest//:main",],
)
