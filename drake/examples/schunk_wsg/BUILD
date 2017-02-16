# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "schunk_wsg_trajectory_generator_state_vector",
    srcs = ["gen/schunk_wsg_trajectory_generator_state_vector.cc"],
    hdrs = ["gen/schunk_wsg_trajectory_generator_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "schunk_wsg_lcm",
    srcs = ["schunk_wsg_lcm.cc"],
    hdrs = ["schunk_wsg_lcm.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__pkg__"],
    deps = [
        ":schunk_wsg_trajectory_generator_state_vector",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcmtypes:schunk",
        "//drake/systems/framework:leaf_system",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "schunk_wsg_lift_test",
    data = [
        ":models",
        "//drake/multibody:models",
    ],
    # This test is prohibitively slow with --compilation_mode=dbg.
    disable_in_compilation_mode_dbg = True,
    deps = [
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:trajectory_source",
    ],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

cpplint()
