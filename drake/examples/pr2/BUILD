# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "robot_state_lcm",
    srcs = ["src/utils/robot_state_lcm.cc"],
    hdrs = ["src/utils/robot_state_lcm.h"],
    deps = [
        "//drake/lcmtypes:robot_state",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "plan_status_lcm",
    srcs = ["src/utils/plan_status_lcm.cc"],
    hdrs = ["src/utils/plan_status_lcm.h"],
    deps = [
        "@lcmtypes_robotlocomotion",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["src/utils/world_sim_tree_builder.cc"],
    hdrs = ["src/utils/world_sim_tree_builder.h"],
    deps = [
        "//drake/common:unused",
	"//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

drake_cc_library(
    name = "robot_plan_interpolator",
    srcs = ["src/utils/robot_plan_interpolator.cc"],
    hdrs = ["src/utils/robot_plan_interpolator.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "pr2_simulator",
    srcs = ["src/simulator/pr2_simulator.cc"],
    deps = [
        ":robot_state_lcm",
	":robot_plan_interpolator",
        ":world_sim_tree_builder",
        ":plan_status_lcm",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
	"//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controller",
	"//drake/systems/analysis:semi_explicit_euler_integrator",
	"//drake/systems/framework:diagram_builder",
	"//drake/common:find_resource",
	"@lcmtypes_robotlocomotion",
    ],
)

install_data()

add_lint_tests()
