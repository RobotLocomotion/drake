# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball",
    srcs = ["ball.cc"],
    hdrs = [
        "ball.h",
        "ball-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/systems/framework:leaf_system",
    ],
)

cc_library(
    name = "bouncing_ball",
    srcs = ["bouncing_ball.cc"],
    hdrs = [
        "bouncing_ball.h",
        "bouncing_ball-inl.h",
    ],
    linkstatic = 1,
    deps = [
        ":ball",
        "//drake/common",
    ],
)

# === test/ ===

cc_unit_test(
    name = "ball_test",
    deps = [
        ":ball",
    ],
)

cc_unit_test(
    name = "bouncing_ball_test",
    deps = [
        ":bouncing_ball",
    ],
)
