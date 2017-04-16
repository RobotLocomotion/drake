# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "simple_tree_visualizer",
    srcs = ["simple_tree_visualizer.cc"],
    hdrs = ["simple_tree_visualizer.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        "//drake/common",
        "//drake/lcm",
        "//drake/systems/framework",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant:create_load_robot_message",
        "//drake/lcmtypes:viewer",
        "//drake/systems/rendering:drake_visualizer_client",
    ]
)

drake_cc_binary(
    name = "simple_tree_visualizer_demo",
    srcs = ["simple_tree_visualizer_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    test_rule_args = ["--num_configurations=1"],
    deps = [
        ":simple_tree_visualizer",
        "//drake/common",
        "@gflags//:gflags",
        "//drake/lcm",
        "//drake/systems/framework",
        "//drake/multibody/parsers",        
    ],
)

cpplint()
