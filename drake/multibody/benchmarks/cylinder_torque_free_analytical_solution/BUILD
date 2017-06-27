# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

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

# === test/ ===

drake_cc_googletest(
    name = "cylinder_torque_free_analytical_solution",
    size = "medium",
    data = [":models"],
    deps = [
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

cpplint()
