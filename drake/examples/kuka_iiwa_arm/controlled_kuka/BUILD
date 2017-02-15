# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "inverse_dynamics_controller_config",
    srcs = glob(["inverse_dynamics_controller_config/*"]),
)

drake_cc_library(
    name = "make_demo_plan",
    srcs = ["make_demo_plan.cc"],
    hdrs = ["make_demo_plan.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_binary(
    name = "kuka_inverse_dynamics_demo",
    srcs = ["kuka_inverse_dynamics_demo.cc"],
    data = [
        ":inverse_dynamics_controller_config/iiwa.alias_groups",
        ":inverse_dynamics_controller_config/iiwa.id_controller_config",
    ],
    tags = [
        "gurobi",
        "snopt",
    ],
    deps = [
        ":make_demo_plan",
        "//drake/examples/QPInverseDynamicsForHumanoids/system:id_controlled_iiwa",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "@gflags//:gflags",
    ],
)

cpplint()
