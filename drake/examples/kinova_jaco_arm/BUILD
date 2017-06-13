# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(
    default_visibility = [":__subpackages__"],
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
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "run_passive_jaco_demo",
    srcs = ["run_passive_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/manipulation/models/jaco_description:models",
    ],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":jaco_common",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "run_setpose_jaco_demo",
    srcs = ["run_setpose_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/manipulation/models/jaco_description:models",
    ],
    tags = ["snopt"],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":jaco_common",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "run_controlled_jaco_demo",
    srcs = ["run_controlled_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/manipulation/models/jaco_description:models",
    ],
    tags = ["snopt"],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":jaco_common",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcm",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
