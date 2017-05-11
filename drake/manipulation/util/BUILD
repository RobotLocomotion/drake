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
        "@bot_core_lcmtypes",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "robot_state_msg_translator_test",
    # TODO(siyuan): we should eventually move the models for test outside of
    # /examples.
    data = [
        "//drake/examples/Valkyrie:models",
        "//drake/examples/schunk_wsg:models",
        "//drake/manipulation/models/iiwa_description:models",
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
