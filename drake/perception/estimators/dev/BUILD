# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "models",
    testonly = 1,
    srcs = glob([
        "**/*.urdf",
    ]),
    visibility = ["//visibility:private"],
)

drake_cc_library(
    name = "scene",
    srcs = ["scene.cc"],
    hdrs = ["scene.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
    ],
)

drake_cc_library(
    name = "articulated_icp",
    srcs = ["articulated_icp.cc"],
    hdrs = ["articulated_icp.h"],
    deps = [
        ":scene",
        "//drake/solvers:cost",
    ],
)

drake_cc_googletest(
    name = "articulated_icp_test",
    data = [
        ":models",
    ],
    deps = [
        ":articulated_icp",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/solvers:mathematical_program",
        "@lcmtypes_bot2_core",
    ],
)

add_lint_tests()
