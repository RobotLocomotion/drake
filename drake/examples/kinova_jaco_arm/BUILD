# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

package(
    default_visibility = [":__subpackages__"],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "jaco_common",
    srcs = [
        "jaco_common.cc",
    ],
    hdrs = [
        "jaco_common.h",
    ],
    deps = [
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "run_passive_jaco_demo",
    srcs = ["run_passive_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/jaco_description:models",
    ],
    test_rule_args = ["--duration=0.5"],
    test_rule_size = "medium",
    deps = [
        "@gflags//:gflags",
        ":jaco_common",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:state_feedback_controller_base",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_binary(
    name = "run_setpose_jaco_demo",
    srcs = ["run_setpose_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/jaco_description:models",
    ],
    tags = ["snopt"],
    deps = [
        "@gflags//:gflags",
        ":jaco_common",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:state_feedback_controller_base",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cpplint()
