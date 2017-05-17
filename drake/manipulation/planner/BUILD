# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "relaxing_ik_wrapper",
    srcs = ["relaxing_ik_wrapper.cc"],
    hdrs = ["relaxing_ik_wrapper.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "relaxing_ik_wrapper_test",
    size = "medium",
    srcs = ["test/relaxing_ik_wrapper_test.cc"],
    data = [
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":relaxing_ik_wrapper",
    ],
)
