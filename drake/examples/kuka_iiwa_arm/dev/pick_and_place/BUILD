# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "kuka_pick_and_place",
    srcs = [
        "action.cc",
        "pick_and_place_common.cc",
        "world_state.cc",
    ],
    hdrs = [
        "action.h",
        "pick_and_place_common.h",
        "world_state.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm/dev:iiwa_ik_planner",
        "//drake/lcmtypes:schunk",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/util:lcm_util",
    ],
)

drake_cc_binary(
    name = "pick_and_place_state_machine",
    srcs = [
        "pick_and_place_state_machine.cc",
    ],
    deps = [
        ":kuka_pick_and_place",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/dev:iiwa_ik_planner",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/util:lcm_util",
    ],
)

# === test/ ===
