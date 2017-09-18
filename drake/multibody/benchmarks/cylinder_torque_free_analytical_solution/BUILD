# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "models",
    testonly = 1,
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

drake_cc_library(
    name = "torque_free_cylinder_exact_solution",
    srcs = [
        "torque_free_cylinder_exact_solution.cc",
    ],
    hdrs = [
        "torque_free_cylinder_exact_solution.h",
    ],
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "cylinder_torque_free_analytical_solution",
    size = "medium",
    data = [":models"],
    deps = [
        ":torque_free_cylinder_exact_solution",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/math:autodiff",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:runge_kutta3_integrator",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

add_lint_tests()
