# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "constraint_relaxing_ik",
    srcs = ["constraint_relaxing_ik.cc"],
    hdrs = ["constraint_relaxing_ik.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "constraint_relaxing_ik_test",
    size = "medium",
    srcs = ["test/constraint_relaxing_ik_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    # TODO(sam.creasey) This test currently times out on Mac with
    # IPOPT.  Once IPOPT is tuned to provide better/faster IK results
    # (see https://github.com/RobotLocomotion/drake/issues/3128 ) we
    # should enable this test.
    tags = ["snopt"],
    deps = [
        ":constraint_relaxing_ik",
        "//drake/common:find_resource",
    ],
)

add_lint_tests()
