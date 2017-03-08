# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library", "drake_cc_binary")

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
    name = "joint_level_controller_system",
    srcs = ["joint_level_controller_system.cc"],
    hdrs = ["joint_level_controller_system.h"],
    deps = [
        "@bot_core_lcmtypes//:lib",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "plan_eval_system",
    srcs = ["plan_eval_system.cc"],
    hdrs = ["plan_eval_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/systems/framework:leaf_system",
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

drake_cc_binary(
    name = "valkyrie_balancing_controller_system",
    srcs = [
        "valkyrie_balancing_controller_system.cc",
    ],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",
        "//drake/examples/Valkyrie:models",
    ],
    deps = [
        "@bot_core_lcmtypes//:lib",
        ":joint_level_controller_system",
        ":plan_eval_system",
        ":qp_controller_system",
        ":robot_state_decoder_system",
        "//drake/examples/Valkyrie:valkyrie_constants",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_value_source",
    ],
)

cpplint()
