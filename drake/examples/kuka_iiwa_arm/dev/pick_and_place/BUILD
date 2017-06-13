# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
)

drake_cc_binary(
    name = "pick_and_place_demo",
    srcs = [
        "pick_and_place_demo.cc",
    ],
    deps = [
        "//drake/examples/kuka_iiwa_arm/pick_and_place",
        "//drake/examples/kuka_iiwa_arm:iiwa_common",
        "//drake/examples/kuka_iiwa_arm:iiwa_lcm",
        "//drake/util:lcm_util",
    ],
)

# === test/ ===

cpplint()
