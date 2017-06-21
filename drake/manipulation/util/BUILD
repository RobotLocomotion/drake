# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "robot_state_msg_translator",
    srcs = ["robot_state_msg_translator.cc"],
    hdrs = ["robot_state_msg_translator.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/util",
        "//drake/util:lcm_util",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_library(
    name = "trajectory_utils",
    srcs = [
        "trajectory_utils.cc",
    ],
    hdrs = [
        "trajectory_utils.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories:piecewise_quaternion",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "robot_state_msg_translator_test",
    # TODO(siyuan): we should eventually move the models for test outside of
    # /examples.
    data = [
        "//drake/examples/Valkyrie:models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":robot_state_msg_translator",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/math:geometric_transform",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "trajectory_utils_test",
    deps = [
        ":trajectory_utils",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
