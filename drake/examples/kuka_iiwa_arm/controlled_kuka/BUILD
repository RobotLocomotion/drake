# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

cc_binary(
    name = "controlled_kuka_demo",
    srcs = ["controlled_kuka_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
        "@gflags//:gflags",
    ],
)

cc_binary(
    name = "kuka_cartesian_way_point_demo",
    srcs = ["kuka_cartesian_way_point_demo.cc"],
    data = ["//drake/examples/kuka_iiwa_arm:models"],
    deps = [
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
        "@gflags//:gflags",
    ],
)
