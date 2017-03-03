# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

drake_cc_library(
    name = "kuka_demo_plant_builder",
    hdrs = ["kuka_demo_plant_builder.h"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:trajectory_source",
    ],
)

cc_binary(
    name = "controlled_kuka_demo",
    srcs = ["controlled_kuka_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        ":kuka_demo_plant_builder",
        "//drake/systems/analysis:simulator",
        "@gflags//:gflags",
    ],
)

cc_binary(
    name = "kuka_cartesian_way_point_demo",
    srcs = ["kuka_cartesian_way_point_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        ":kuka_demo_plant_builder",
        "//drake/systems/analysis:simulator",
        "@gflags//:gflags",
    ],
)
