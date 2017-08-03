# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = [
        "//drake/multibody/benchmarks/kuka_iiwa_robot:__subpackages__",
    ],
)

drake_cc_library(
    name = "MG_kuka_robot_lib",
    testonly = 1,
    srcs = [
        "MG_kuka_iiwa_robot.cc",
        "MG_kuka_iiwa_robot_auto_generated.cc",
    ],
    hdrs = [
        "MG_kuka_iiwa_robot.h",
        "MG_kuka_iiwa_robot_auto_generated.h",
    ],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:extract_double",
        "//drake/multibody/multibody_tree/math:spatial_force",
    ],
)

add_lint_tests()
