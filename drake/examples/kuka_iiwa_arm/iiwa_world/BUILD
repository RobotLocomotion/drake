# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["world_sim_tree_builder.cc"],
    hdrs = ["world_sim_tree_builder.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

drake_cc_library(
    name = "world_sim_diagram_factory",
    srcs = ["world_sim_diagram_factory.cc"],
    hdrs = ["world_sim_diagram_factory.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:trajectory_source",
    ],
)

drake_cc_binary(
    name = "iiwa_world_demo",
    srcs = ["iiwa_world_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        ":world_sim_diagram_factory",
        ":world_sim_tree_builder",
        "//drake/common",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "controlled_iiwa_world_demo",
    srcs = ["controlled_iiwa_world_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        ":world_sim_diagram_factory",
        ":world_sim_tree_builder",
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "@gflags//:gflags",
    ],
)

cpplint()
