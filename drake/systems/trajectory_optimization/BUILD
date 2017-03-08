# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

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
        ":direct_trajectory_optimization",
        "//drake/math:autodiff",
    ],
)

drake_cc_library(
    name = "direct_trajectory_optimization",
    srcs = ["direct_trajectory_optimization.cc"],
    hdrs = ["direct_trajectory_optimization.h"],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework",
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
    name = "trajectory_optimization_test",
    deps = [
        ":direct_collocation",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

cpplint()
