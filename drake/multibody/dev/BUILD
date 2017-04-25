# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

drake_cc_library(
    name = "global_inverse_kinematics_test_util",
    testonly = 1,
    srcs = [
        "test/global_inverse_kinematics_test_util.cc",
    ],
    hdrs = [
        "test/global_inverse_kinematics_test_util.h",
    ],
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:global_inverse_kinematics",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "@gtest//:main",
    ],
)

# Temporarily disabled because we have no Gurobi license in CI. #5862
#drake_cc_googletest(
#    name = "global_inverse_kinematics_test",
#    size = "medium",
#    srcs = ["test/global_inverse_kinematics_test.cc"],
#    data = [
#        "//drake/manipulation/models/iiwa_description:models",
#    ],
#    tags = ["gurobi"],
#    deps = [
#        ":global_inverse_kinematics_test_util",
#    ],
#)

# Temporarily disabled because we have no Gurobi license in CI. #5862
#drake_cc_googletest(
#    name = "global_inverse_kinematics_reachable_test",
#    size = "medium",
#    srcs = ["test/global_inverse_kinematics_reachable_test.cc"],
#    data = [
#        "//drake/manipulation/models/iiwa_description:models",
#    ],
#    tags = ["gurobi"],
#    deps = [
#        ":global_inverse_kinematics_test_util",
#    ],
#)

cpplint()
