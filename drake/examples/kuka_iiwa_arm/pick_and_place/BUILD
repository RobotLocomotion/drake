# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
)

drake_cc_library(
    name = "pick_and_place",
    srcs = [
        "action.cc",
        "pick_and_place_state_machine.cc",
        "world_state.cc",
    ],
    hdrs = [
        "action.h",
        "pick_and_place_state_machine.h",
        "world_state.h",
    ],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcmtypes:schunk",
        "//drake/manipulation/planner:constraint_relaxing_ik",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/util:lcm_util",
        "@bot_core_lcmtypes",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_googletest(
    name = "pick_and_place_state_machine_test",
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":pick_and_place",
    ],
)

drake_cc_googletest(
    name = "world_state_test",
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":pick_and_place",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
