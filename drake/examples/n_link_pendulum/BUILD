# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "n_link_pendulum_plant",
    srcs = ["n_link_pendulum_plant.cc"],
    hdrs = ["n_link_pendulum_plant.h"],
    deps = [
        "//drake/common:default_scalars",
        "//drake/geometry:geometry_ids",
        "//drake/geometry:geometry_system",
        "//drake/math:geometric_transform",
        "//drake/multibody/multibody_tree",
        "//drake/systems/framework",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "n_link_pendulum_run_dynamics",
    srcs = ["n_link_pendulum_run_dynamics.cc"],
    deps = [
        ":n_link_pendulum_plant",
        "//drake/geometry:geometry_system",
        "//drake/geometry:geometry_visualization",
        "//drake/systems/analysis:implicit_euler_integrator",
        "//drake/systems/analysis:runge_kutta2_integrator",
        "//drake/systems/analysis:runge_kutta3_integrator",
        "//drake/systems/analysis:semi_explicit_euler_integrator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
    ],
)

drake_cc_googletest(
    name = "n_link_pendulum_to_autodiff",
    deps = [
        ":n_link_pendulum_plant",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/systems/primitives:linear_system",
    ],
)

add_lint_tests()
