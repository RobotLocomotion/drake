# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library", "drake_cc_binary")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "robot_state_decoder_system",
    srcs = ["robot_state_decoder_system.cc"],
    hdrs = ["robot_state_decoder_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids:lcm_utils",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "state_to_humanoid_status",
    srcs = ["state_to_humanoid_status.cc"],
    hdrs = ["state_to_humanoid_status.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "joint_level_controller_system",
    srcs = ["joint_level_controller_system.cc"],
    hdrs = ["joint_level_controller_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
        "@bot_core_lcmtypes//:lib",
    ],
)

drake_cc_library(
    name = "atlas_joint_level_controller_system",
    srcs = ["atlas_joint_level_controller_system.cc"],
    hdrs = ["atlas_joint_level_controller_system.h"],
    deps = [
        ":joint_level_controller_system",
        "@bot_core_lcmtypes//:lib",
    ],
)

drake_cc_library(
    name = "discrete_time_plan_eval_system",
    srcs = ["discrete_time_plan_eval_system.cc"],
    hdrs = ["discrete_time_plan_eval_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "iiwa_servo_system",
    srcs = ["kuka_servo_system.cc"],
    hdrs = ["kuka_servo_system.h"],
    deps = [
        ":discrete_time_plan_eval_system",
    ],
)

drake_cc_library(
    name = "humanoid_plan_eval_system",
    srcs = ["humanoid_plan_eval_system.cc"],
    hdrs = ["humanoid_plan_eval_system.h"],
    deps = [
        ":discrete_time_plan_eval_system",
    ],
)

drake_cc_library(
    name = "qp_controller_system",
    srcs = ["qp_controller_system.cc"],
    hdrs = ["qp_controller_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "id_controlled_iiwa",
    srcs = [],
    hdrs = ["kuka_inverse_dynamics_servo.h"],
    deps = [
        ":iiwa_servo_system",
        ":joint_level_controller_system",
        ":qp_controller_system",
        ":state_to_humanoid_status",
    ],
)

drake_cc_binary(
    name = "valkyrie_balancing_controller_system",
    srcs = [
        "valkyrie_balancing_controller_system.cc",
    ],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",
        "//drake/examples/Valkyrie:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":atlas_joint_level_controller_system",
        ":humanoid_plan_eval_system",
        ":qp_controller_system",
        ":robot_state_decoder_system",
        "//drake/examples/Valkyrie:valkyrie_constants",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_value_source",
        "@bot_core_lcmtypes//:lib",
    ],
)

cpplint()
