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
    name = "pendulum_run_swing_up",
    srcs = ["pendulum_run_swing_up.cc"],
    data = [":models"],
    deps = [
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
        "@gflags//:gflags",
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
    name = "pendulum_trajectory_optimization_test",
    deps = [
        ":pendulum_swing_up",
        "//drake/common:eigen_matrix_compare",
    ],
)

sh_test(
    name = "pendulum_run_swing_up_test",
    srcs = ["//tools:exec.sh"],
    args = [
        "$(location :pendulum_run_swing_up)",
        "--target_realtime_rate=0.0",
    ],
    data = [":pendulum_run_swing_up"],
)

filegroup(
    name = "models",
    srcs = glob([
        "Pendulum.urdf",
    ]),
    visibility = ["//visibility:public"],
)

cpplint()
