# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "world_sim_tree_builder",
    srcs = ["world_sim_tree_builder.cc"],
    hdrs = ["world_sim_tree_builder.h"],
    visibility = ["//drake/examples/kinova_jaco_arm:__subpackages__"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

cpplint()
