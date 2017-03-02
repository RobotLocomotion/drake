# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "sliding_bricks",
    srcs = ["sliding_bricks.cc"],
    data = ["sliding_brick.urdf"],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "rigid_gripper",
    srcs = ["rigid_gripper.cc"],
    data = [
        "rigid_gripper.urdf",
        "//drake/multibody:models",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcm:lcm",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant:rigid_body_plant",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/systems/analysis",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:trajectory_source",
        "@gflags//:gflags",
    ],
)

cpplint()
