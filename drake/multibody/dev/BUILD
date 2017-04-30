# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:gurobi.bzl", "gurobi_test_tags")

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
        "@gtest//:main",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody:global_inverse_kinematics",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "@gtest//:without_main",
    ],
)

drake_cc_googletest(
    name = "global_inverse_kinematics_test",
    size = "medium",
    srcs = ["test/global_inverse_kinematics_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    # Excluding because an assertion fails in LLVM code. Issue #6179.
    tags = gurobi_test_tags() + ["no_tsan"],
    deps = [
        ":global_inverse_kinematics_test_util",
    ],
)

drake_cc_googletest(
    name = "global_inverse_kinematics_reachable_test",
    size = "medium",
    srcs = ["test/global_inverse_kinematics_reachable_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    # Excluding because an assertion fails in LLVM code. Issue #6179.
    tags = gurobi_test_tags() + ["no_tsan"],
    deps = [
        ":global_inverse_kinematics_test_util",
    ],
)

drake_cc_googletest(
    name = "global_inverse_kinematics_collision_avoidance_test",
    size = "medium",
    srcs = ["test/global_inverse_kinematics_collision_avoidance_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    # Excluding because an assertion fails in LLVM code. Issue #6179.
    tags = gurobi_test_tags() + ["no_tsan"],
    deps = [
        ":global_inverse_kinematics_test_util",
    ],
)

drake_cc_googletest(
    name = "global_inverse_kinematics_feasible_posture_test",
    srcs = ["test/global_inverse_kinematics_feasible_posture_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = ["gurobi"],
    deps = [
        ":global_inverse_kinematics_test_util",
    ],
)

cpplint()
