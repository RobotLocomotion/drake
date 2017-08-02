# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "ball",
    srcs = ["ball.cc"],
    hdrs = [
        "ball.h",
        "ball-inl.h",
    ],
    deps = [
        "//drake/common",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "bouncing_ball",
    srcs = ["bouncing_ball.cc"],
    hdrs = [
        "bouncing_ball.h",
        "bouncing_ball-inl.h",
    ],
    deps = [
        ":ball",
        "//drake/common",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "ball_test",
    deps = [
        ":ball",
    ],
)

drake_cc_googletest(
    name = "bouncing_ball_test",
    deps = [
        ":bouncing_ball",
        "//drake/systems/analysis",
    ],
)

add_lint_tests()
