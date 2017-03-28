# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "iiwa_ik_planner",
    srcs = ["iiwa_ik_planner.cc"],
    hdrs = ["iiwa_ik_planner.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "iiwa_ik_test",
    size = "medium",
    srcs = ["test/iiwa_ik_test.cc"],
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_ik_planner",
    ],
)
