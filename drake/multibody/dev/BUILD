# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)

drake_cc_googletest(
    name = "global_inverse_kinematics_test",
    srcs = ["test/global_inverse_kinematics_test.cc"],
    size = "large",
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    tags = ["gurobi"],
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:global_inverse_kinematics",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
    ],
)

cpplint()
