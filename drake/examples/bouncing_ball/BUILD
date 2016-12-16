# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

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

cc_test(
    name = "ball_test",
    size = "small",
    srcs = ["test/ball_test.cc"],
    deps = [
        ":ball",
        "@gtest//:main",
    ],
)

cc_test(
    name = "bouncing_ball_test",
    size = "small",
    srcs = ["test/bouncing_ball_test.cc"],
    deps = [
        ":bouncing_ball",
        "@gtest//:main",
    ],
)
