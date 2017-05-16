# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
)

drake_cc_library(
    name = "action_primitives",
    srcs = [
        "action_primitive_base.cc",
        "gripper_action.cc",
        "iiwa_move.cc",
    ],
    hdrs = [
        "action_primitive_base.h",
        "action_primitives_common.h",
        "gripper_action.h",
        "iiwa_move.h",
    ],
    visibility = ["//drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place:__subpackages__"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/lcm",
        "//drake/lcmtypes:iiwa",
        "//drake/lcmtypes:schunk",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "//drake/util:lcm_util",
        "@bot_core_lcmtypes",
        "@robotlocomotion_lcmtypes",
    ],
)

cpplint()
