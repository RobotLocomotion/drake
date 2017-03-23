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
    data = [":models"],
    deps = [
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:eigen_types",
        "//drake/math:quaternion",
        "//drake/math:autodiff",
        "//drake/multibody/joints:floating_base_types",
        "//drake/multibody/parsers:urdf_parser",
        "//drake/multibody/rigid_body_plant:rigid_body_plant",
        "//drake/systems/analysis:runge_kutta3_integrator",
        "//drake/systems/analysis:runge_kutta3_integrator-inl",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cpplint()
