# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

drake_cc_library(
    name = "action_primitives",
    srcs = [
        "action_primitive_base.cc",
        "iiwa_move.cc",
        "gripper_action.cc",
      ],
    hdrs = [
        "action_primitive_base.h",
        "iiwa_move.h",
        "gripper_action.h",
        "action_primitives_common.h",
    ],
    visibility = ["//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:__subpackages__"],
    deps = [
        "//drake/examples/kuka_iiwa_arm/dev:iiwa_ik_planner",
      #  "//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:kuka_pick_and_place",
        "//drake/examples/kuka_iiwa_arm:iiwa_plan_source",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcmtypes:schunk",
        "//drake/lcmtypes:iiwa",
        "//drake/lcm",
        "//drake/util:lcm_util",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
        "//drake/systems/analysis",
    ],
)