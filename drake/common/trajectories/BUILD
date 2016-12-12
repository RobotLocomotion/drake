# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

# TODO(jwnimmer-tri) Move code to a trajectory.cc file
cc_component(
    name = "trajectory",
    srcs = [],
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

cc_component(
    name = "piecewise_polynomial_trajectory",
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

cc_unit_test(
    name = "piecewise_polynomial_test",
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "exponential_plus_piecewise_polynomial_test",
    deps = [
        ":piecewise_polynomial",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "piecewise_polynomial_trajectory_test",
    deps = [
        ":piecewise_polynomial_trajectory",
        ":random_piecewise_polynomial",
        "//drake/common:eigen_matrix_compare",
    ],
)
