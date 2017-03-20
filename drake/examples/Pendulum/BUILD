# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library", "drake_cc_binary")

drake_cc_library(
    name = "pendulum_state_vector",
    srcs = ["gen/pendulum_state_vector.cc"],
    hdrs = ["gen/pendulum_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "pendulum_plant",
    srcs = ["pendulum_plant.cc"],
    hdrs = ["pendulum_plant.h"],
    deps = [
        ":pendulum_state_vector",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "pendulum_swing_up",
    srcs = ["pendulum_swing_up.cc"],
    hdrs = ["pendulum_swing_up.h"],
    deps = [
        ":pendulum_plant",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

drake_cc_binary(
    name = "pendulum_run_dynamics",
    srcs = ["pendulum_run_dynamics.cc"],
    add_test_rule = 1,
    data = [":models"],
    deps = [
        ":pendulum_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_binary(
    name = "pendulum_run_energy_shaping",
    srcs = ["pendulum_run_energy_shaping.cc"],
    add_test_rule = 1,
    data = [":models"],
    deps = [
        ":pendulum_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "pendulum_run_lqr",
    srcs = ["pendulum_run_lqr.cc"],
    add_test_rule = 1,
    data = [":models"],
    deps = [
        ":pendulum_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "pendulum_run_swing_up",
    srcs = ["pendulum_run_swing_up.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--target_realtime_rate=0.0"],
    deps = [
        "@gflags//:gflags",
        "pendulum_swing_up",
        "//drake/common:drake_path",
        "//drake/common:is_approx_equal_abstol",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework",
        "//drake/systems/primitives:trajectory_source",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "pendulum_urdf_dynamics_test",
    data = ["Pendulum.urdf"],
    deps = [
        ":pendulum_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "pendulum_dynamic_constraint_test",
    deps = [
        ":pendulum_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

drake_cc_googletest(
    name = "pendulum_plant_test",
    deps = [
        ":pendulum_plant",
        "//drake/common:autodiff",
    ],
)

drake_cc_googletest(
    name = "pendulum_trajectory_optimization_test",
    deps = [
        ":pendulum_swing_up",
        "//drake/common:eigen_matrix_compare",
    ],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

cpplint()
