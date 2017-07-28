# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "robot_state_msg_translator",
    srcs = ["robot_state_msg_translator.cc"],
    hdrs = ["robot_state_msg_translator.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/util",
        "//drake/util:lcm_util",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_library(
    name = "simple_tree_visualizer",
    srcs = ["simple_tree_visualizer.cc"],
    hdrs = ["simple_tree_visualizer.h"],
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

drake_cc_library(
    name = "trajectory_utils",
    srcs = [
        "trajectory_utils.cc",
    ],
    hdrs = [
        "trajectory_utils.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
    ],
)

drake_cc_binary(
    name = "simple_tree_visualizer_demo",
    srcs = ["simple_tree_visualizer_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    test_rule_args = ["--num_configurations=1"],
    deps = [
        ":simple_tree_visualizer",
        "//drake/common",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_library(
    name = "moving_average_filter",
    srcs = ["moving_average_filter.cc"],
    hdrs = ["moving_average_filter.h"],
    deps = ["//drake/common"],
)

# === test/ ===

drake_cc_googletest(
    name = "robot_state_msg_translator_test",
    # TODO(siyuan): we should eventually move the models for test outside of
    # /examples.
    data = [
        "//drake/examples/valkyrie:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":robot_state_msg_translator",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/math:geometric_transform",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "trajectory_utils_test",
    deps = [
        ":trajectory_utils",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "moving_average_filter_test",
    srcs = ["test/moving_average_filter_test.cc"],
    deps = [
        ":moving_average_filter",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
