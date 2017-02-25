# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
    "drake_cc_googletest",
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "iiwa_ik_planner",
    srcs = ["iiwa_ik_planner.cc"],
    hdrs = ["iiwa_ik_planner.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "kuka_pick_and_place",
    srcs = ["pick_and_place_state.cc"],
    hdrs = ["pick_and_place_state.h"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/util:lcm_util",
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
    ],
)

drake_cc_binary(
    name = "pick_and_place_state_machine",
    srcs = [
        "pick_and_place_state_machine.cc",
    ],
    deps = [
        ":iiwa_ik_planner",
        ":kuka_pick_and_place",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/common/trajectories:piecewise_quaternion",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/util:lcm_util",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "iiwa_ik_test",
    size = "medium",
    srcs = ["test/iiwa_ik_test.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    deps = [
        ":iiwa_ik_planner",
    ],
)
