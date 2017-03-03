# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "sliding_bricks",
    srcs = ["sliding_bricks.cc"],
    data = ["sliding_brick.urdf"],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm:lcm",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody/rigid_body_plant:rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "bowling_ball",
    srcs = ["bowling_ball.cc"],
    data = [
        ":bowling_models",
    ],
    deps = [
        "//drake/common:drake_path",
        "//drake/lcm:lcm",
        "//drake/multibody/parsers:parsers",
        "//drake/multibody/rigid_body_plant:rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/systems/analysis",
        "//drake/systems/lcm:lcm",
        "@gflags//:gflags",
    ],
)

filegroup(
    name = "bowling_models",
    srcs = [
        "bowling_ball.urdf",
        "pin.urdf",
        "pin.obj",
    ],
)

cpplint()
