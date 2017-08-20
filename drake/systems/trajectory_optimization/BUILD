# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "multiple_shooting",
    srcs = ["multiple_shooting.cc"],
    hdrs = ["multiple_shooting.h"],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "direct_collocation",
    srcs = [
        "direct_collocation.cc",
        "direct_collocation_constraint.cc",
    ],
    hdrs = [
        "direct_collocation.h",
        "direct_collocation_constraint.h",
    ],
    deps = [
        ":multiple_shooting",
        "//drake/math:autodiff",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "direct_collocation_constraint_test",
    deps = [
        ":direct_collocation",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "multiple_shooting_test",
    size = "small",
    deps = [
        ":multiple_shooting",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

drake_cc_googletest(
    name = "direct_collocation_test",
    # Test size increased to not timeout when run with Valgrind.
    size = "medium",
    deps = [
        ":direct_collocation",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/systems/primitives:linear_system",
    ],
)

add_lint_tests()
