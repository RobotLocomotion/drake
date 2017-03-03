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
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

# A test to guarantee the example doesn't fall out of sync with Drake.
sh_test(
    name = "sliding_bricks_test",
    srcs = ["//tools:exec.sh"],
    args = [
        "$(location :sliding_bricks)",
        " --sim_duration=0.01",
        " --no_playback=true",
    ],
    data = [":sliding_bricks"],
)

cpplint()
