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

drake_cc_library(
    name = "test_util",
    testonly = 1,
    srcs = ["test/test_util.cc"],
    hdrs = ["test/test_util.h"],
    deps = [
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/lcm",
        "@lcmtypes_bot2_core",
    ],
)

filegroup(
    name = "test_models",
    testonly = 1,
    srcs = glob([
        "test/**/*.urdf",
    ]),
    visibility = ["//visibility:private"],
)

drake_cc_googletest(
    name = "articulated_icp_test",
    data = [
        ":test_models",
    ],
    deps = [
        ":articulated_icp",
        ":test_util",
        "//drake/common:find_resource",
        "//drake/lcmtypes:viewer",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:create_load_robot_message",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/solvers:mathematical_program",
    ],
)

add_lint_tests()
