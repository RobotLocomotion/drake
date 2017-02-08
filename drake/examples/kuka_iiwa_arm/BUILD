# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

drake_cc_library(
    name = "iiwa_common",
    srcs = ["iiwa_common.cc"],
    hdrs = ["iiwa_common.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "iiwa_lcm",
    srcs = ["iiwa_lcm.cc"],
    hdrs = ["iiwa_lcm.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "iiwa_wsg_simulation",
    srcs = ["iiwa_wsg_simulation.cc"],
    deps = [
        ":iiwa_lcm",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/examples/schunk_wsg:schunk_wsg_lcm",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:gravity_compensator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/primitives:multiplexer",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "kuka_simulation",
    srcs = ["kuka_simulation.cc"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:gravity_compensator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:multiplexer",
        "@gflags//:gflags",
    ],
)

cc_binary(
    name = "kuka_ik_demo",
    srcs = ["kuka_ik_demo.cc"],
    deps = [
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
    ],
)

cpplint()
