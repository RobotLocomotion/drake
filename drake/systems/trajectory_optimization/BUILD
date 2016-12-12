# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "direct_collocation",
    testonly = 1,
    srcs = [
        "direct_collocation.cc",
        "direct_collocation_constraint.cc",
    ],
    hdrs = [
        "direct_collocation.h",
        "direct_collocation_constraint.h",
    ],
    linkstatic = 1,
    deps = [
        ":direct_trajectory_optimization",
        "//drake/math:autodiff",
    ],
)

cc_component(
    name = "direct_trajectory_optimization",
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_unit_test(
    name = "direct_collocation_constraint_test",
    deps = [
        ":direct_collocation",
        "//drake/common:eigen_matrix_compare",
    ],
)
