# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

# TODO(jwnimmer-tri) Move code to a trajectory.cc file
cc_library(
    name = "trajectory",
    hdrs = ["trajectory.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "piecewise_polynomial",
    srcs = [
        "exponential_plus_piecewise_polynomial.cc",
        "piecewise_function.cc",
        "piecewise_polynomial.cc",
        "piecewise_polynomial_base.cc",
    ],
    hdrs = [
        "exponential_plus_piecewise_polynomial.h",
        "piecewise_function.h",
        "piecewise_polynomial.h",
        "piecewise_polynomial_base.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:polynomial",
    ],
)

cc_library(
    name = "piecewise_polynomial_trajectory",
    srcs = ["piecewise_polynomial_trajectory.cc"],
    hdrs = ["piecewise_polynomial_trajectory.h"],
    linkstatic = 1,
    deps = [
        ":piecewise_polynomial",
        ":trajectory",
        "//drake/common",
    ],
)

# === test/ ===

cc_library(
    name = "random_piecewise_polynomial",
    testonly = 1,
    hdrs = ["test/random_piecewise_polynomial.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:random_polynomial_matrix",
    ],
)

cc_test(
    name = "piecewise_polynomial_test",
    size = "small",
    srcs = ["test/piecewise_polynomial_test.cc"],
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "exponential_plus_piecewise_polynomial_test",
    size = "small",
    srcs = [
        "test/exponential_plus_piecewise_polynomial_test.cc",
        "test/random_piecewise_polynomial.h",
    ],
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "piecewise_polynomial_trajectory_test",
    size = "small",
    srcs = ["test/piecewise_polynomial_trajectory_test.cc"],
    deps = [
        ":piecewise_polynomial_trajectory",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)
