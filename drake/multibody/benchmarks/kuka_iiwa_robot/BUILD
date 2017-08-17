# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = [":__subpackages__"])

drake_cc_library(
    name = "kuka_iiwa_robot_library",
    testonly = 1,
    srcs = [
    ],
    hdrs = [
        "drake_kuka_iiwa_robot.h",
    ],
    deps = [
        "//drake/math:geometric_transform",
        "//drake/multibody/multibody_tree",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "kuka_iiwa_robot_kinematics_test",
    size = "small",
    deps = [
        ":kuka_iiwa_robot_library",
        "//drake/multibody/benchmarks/kuka_iiwa_robot/MG:MG_kuka_robot_lib",
    ],
)

add_lint_tests()
