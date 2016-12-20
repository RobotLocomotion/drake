# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

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
    name = "piecewise_function",
    srcs = ["piecewise_function.cc"],
    hdrs = ["piecewise_function.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

cc_library(
    name = "piecewise_polynomial",
    srcs = [
        "exponential_plus_piecewise_polynomial.cc",
        "piecewise_polynomial.cc",
        "piecewise_polynomial_base.cc",
    ],
    hdrs = [
        "exponential_plus_piecewise_polynomial.h",
        "piecewise_polynomial.h",
        "piecewise_polynomial_base.h",
    ],
    linkstatic = 1,
    deps = [
        ":piecewise_function",
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

cc_library(
    name = "piecewise_quaternion",
    srcs = ["piecewise_quaternion.cc"],
    hdrs = ["piecewise_quaternion.h"],
    linkstatic = 1,
    deps = [
        ":piecewise_function",
        "//drake/common",
        "//drake/math:geometric_transform",
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

cc_googletest(
    name = "piecewise_function_test",
    deps = [
        ":piecewise_function",
    ],
)

cc_googletest(
    name = "piecewise_generation_test",
    srcs = ["test/piecewise_polynomial_generation_test.cc"],
    deps = [
        ":piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "piecewise_polynomial_test",
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "exponential_plus_piecewise_polynomial_test",
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "piecewise_polynomial_trajectory_test",
    deps = [
        ":piecewise_polynomial_trajectory",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "piecewise_quaternion_test",
    deps = [
        ":piecewise_quaternion",
        "//drake/common:eigen_matrix_compare",
        "//drake/util",
    ],
)
