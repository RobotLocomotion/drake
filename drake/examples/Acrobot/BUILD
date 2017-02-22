# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "Acrobot.urdf",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "acrobot_state_vector",
    srcs = ["gen/acrobot_state_vector.cc"],
    hdrs = ["gen/acrobot_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "acrobot_lcm",
    srcs = ["acrobot_lcm.cc"],
    hdrs = ["acrobot_lcm.h"],
    deps = [
        ":acrobot_state_vector",
        "//drake/lcmtypes:acrobot",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "acrobot_swing_up",
    srcs = ["acrobot_swing_up.cc"],
    hdrs = ["acrobot_swing_up.h"],
    deps = [
        ":acrobot_plant",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

drake_cc_library(
    name = "acrobot_plant",
    srcs = ["acrobot_plant.cc"],
    hdrs = ["acrobot_plant.h"],
    deps = [
        ":acrobot_state_vector",
        "//drake/multibody/parsers",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/sensors:rotary_encoders",
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up",
    srcs = [
        "acrobot_run_swing_up.cc",
        "acrobot_spong_controller.h",
    ],
    deps = [
        ":acrobot_plant",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up_traj_optimization",
    srcs = ["acrobot_run_swing_up_traj_optimization.cc"],
    deps = [
        ":acrobot_plant",
        ":acrobot_swing_up",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/primitives:trajectory_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_spong_controller_w_lcm",
    srcs = [
        "acrobot_spong_controller.h",
        "acrobot_spong_controller_w_lcm.cc",
    ],
    deps = [
        ":acrobot_lcm",
        ":acrobot_plant",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
    ],
)

drake_cc_binary(
    name = "acrobot_plant_w_lcm",
    srcs = ["acrobot_plant_w_lcm.cc"],
    deps = [
        ":acrobot_lcm",
        ":acrobot_plant",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "@gflags//:gflags",
    ],
)

cpplint()
