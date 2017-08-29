# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "acrobot_multibody_plant",
    srcs = ["acrobot_multibody_plant.cc"],
    hdrs = ["acrobot_multibody_plant.h"],
    deps = [
        "//drake/multibody/multibody_tree/modeler:multibody_modeler",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "acrobot_multibody_run_passive",
    srcs = ["acrobot_multibody_run_passive.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":acrobot_multibody_plant",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:signal_logger",
        "@com_github_gflags_gflags//:gflags",
    ],
)

install_data()

add_lint_tests()
