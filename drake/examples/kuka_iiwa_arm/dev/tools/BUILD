# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_library",
    "drake_cc_googletest",
)

drake_cc_library(
    name = "simple_tree_visualizer",
    srcs = ["simple_tree_visualizer.cc"],
    hdrs = ["simple_tree_visualizer.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        "//drake/common",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/multibody/rigid_body_plant:create_load_robot_message",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/framework",
        "//drake/systems/rendering:drake_visualizer_client",
    ],
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
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "@gflags//:gflags",
    ],
)

drake_cc_library(
    name = "optitrack_pose_translator",
    srcs = ["optitrack_pose_translator.cc"],
    hdrs = ["optitrack_pose_translator.h"],
    visibility =  ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = ["@optitrack_driver//lcmtypes:optitrack_lcmtypes", 
            "//drake/common:common",
            "//drake/util:lcm_util",
            "//drake/math:geometric_transform",
            ":moving_average_filter"
    ],
)
    
drake_cc_library(
    name = "moving_average_filter",
    srcs = ["moving_average_filter.cc"],
    hdrs = ["moving_average_filter.h"],
    deps = ["//drake/common:common"],
)

# === test/ ===

drake_cc_googletest(
    name = "moving_average_filter_test",
    srcs = ["test/moving_average_filter_test.cc"],
    deps = [
        ":moving_average_filter", 
        "//drake/common"],
)

cpplint()
