# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

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

drake_cc_library(
    name = "contact_implicit_constraints",
    srcs = ["contact_implicit_constraints.cc"],
    hdrs = ["contact_implicit_constraints.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/solvers:binding",
        "//drake/solvers:constraint",
        "//drake/solvers:mathematical_program",
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
    # Test size increased to not timeout when run with Valgrind.
    size = "medium",
    deps = [
        ":direct_collocation",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/systems/primitives:linear_system",
    ],
)

drake_cc_googletest(
    name = "contact_implicit_constraints_test",
    deps = [
        ":contact_implicit_constraints",
    ],
)

add_lint_tests()
