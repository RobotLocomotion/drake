# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
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
    name = "acrobot_lcm_msg_handler",
    srcs = [],
    hdrs = ["acrobot_lcm_msg_handler.h"],
    deps = [
        "//drake/lcm",
        "//drake/lcmtypes:acrobot",
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
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/sensors:rotary_encoders",
    ],
)

drake_cc_binary(
    name = "acrobot_lcm_msg_generator",
    srcs = ["acrobot_lcm_msg_generator.cpp"],
    deps = [
        ":acrobot_lcm_msg_handler",
        "//drake/lcm",
        "//drake/lcmtypes:acrobot",
        "//drake/systems/analysis:simulator",
    ],
)

drake_cc_binary(
    name = "acrobot_run_lqr",
    srcs = ["acrobot_run_lqr.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":acrobot_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_run_lqr_w_estimator",
    testonly = 1,
    srcs = ["acrobot_run_lqr_w_estimator.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":acrobot_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/lcm:lcm_call_matlab",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/estimators:kalman_filter",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:linear_system",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/sensors:rotary_encoders",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_run_passive",
    srcs = ["acrobot_run_passive.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":acrobot_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up",
    srcs = [
        "acrobot_run_swing_up.cc",
        "acrobot_spong_controller.h",
    ],
    data = [":models"],
    deps = [
        ":acrobot_plant",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up_traj_optimization",
    srcs = ["acrobot_run_swing_up_traj_optimization.cc"],
    data = [":models"],
    deps = [
        ":acrobot_plant",
        ":acrobot_swing_up",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
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
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
    ],
)

drake_cc_binary(
    name = "acrobot_plant_w_lcm",
    srcs = ["acrobot_plant_w_lcm.cc"],
    data = [":models"],
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

drake_cc_googletest(
    name = "acrobot_urdf_dynamics_test",
    data = [":models"],
    deps = [
        ":acrobot_plant",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

cpplint()
