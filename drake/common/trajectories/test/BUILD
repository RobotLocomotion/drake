# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = [
    ":random_piecewise_polynomial",
    "//drake/common:eigen_matrix_compare",
    "@gtest//:main",
]

cc_library(
    name = "random_piecewise_polynomial",
    hdrs = ["random_piecewise_polynomial.h"],
    deps = ["//drake/common/test:random_polynomial_matrix"],
    testonly = 1,
    linkstatic = 1)

cc_test(
    name = "piecewise_polynomial_test",
    srcs = ["piecewise_polynomial_test.cc"],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial"],
    size = "small")

cc_test(
    name = "exponential_plus_piecewise_polynomial_test",
    srcs = ["exponential_plus_piecewise_polynomial_test.cc", "random_piecewise_polynomial.h"],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial"],
    size = "small")

cc_test(
    name = "piecewise_polynomial_trajectory_test",
    srcs = ["piecewise_polynomial_trajectory_test.cc"],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial_trajectory"],
    size = "small")



