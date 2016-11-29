# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    ":random_piecewise_polynomial",
    "//drake/common:eigen_matrix_compare",
    "@gtest//:main",
]

cc_library(
    name = "random_piecewise_polynomial",
    testonly = 1,
    hdrs = ["random_piecewise_polynomial.h"],
    linkstatic = 1,
    deps = ["//drake/common/test:random_polynomial_matrix"],
)

cc_test(
    name = "piecewise_polynomial_test",
    size = "small",
    srcs = ["piecewise_polynomial_test.cc"],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial"],
)

cc_test(
    name = "exponential_plus_piecewise_polynomial_test",
    size = "small",
    srcs = [
        "exponential_plus_piecewise_polynomial_test.cc",
        "random_piecewise_polynomial.h",
    ],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial"],
)

cc_test(
    name = "piecewise_polynomial_trajectory_test",
    size = "small",
    srcs = ["piecewise_polynomial_trajectory_test.cc"],
    deps = DEPS + ["//drake/common/trajectories:piecewise_polynomial_trajectory"],
)
