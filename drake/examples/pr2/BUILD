# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")
load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "pr2_passive_simulation",
    srcs = ["pr2_passive_simulation.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:semi_explicit_euler_integrator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "pr2_simulation",
    srcs = ["pr2_simulation.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/kuka_iiwa_arm:robot_plan_interpolator",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:semi_explicit_euler_integrator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "pr2_example_plan_sender",
    srcs = ["pr2_example_plan_sender.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "load_pr2_simplified_test",
    srcs = ["test/load_pr2_simplified_test.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
    ],
)

install_data()

add_lint_tests()
