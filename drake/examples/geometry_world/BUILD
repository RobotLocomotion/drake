# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "bouncing_ball_vector",
    srcs = ["gen/bouncing_ball_vector.cc"],
    hdrs = ["gen/bouncing_ball_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "bouncing_ball_plant",
    srcs = ["bouncing_ball_plant.cc"],
    hdrs = ["bouncing_ball_plant.h"],
    deps = [
        ":bouncing_ball_vector",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_binary(
    name = "bouncing_ball_run_dynamics",
    testonly = 1,
    srcs = ["bouncing_ball_run_dynamics.cc"],
    add_test_rule = 1,
    deps = [
        ":bouncing_ball_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
    ],
)

add_lint_tests()
