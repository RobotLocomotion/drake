# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "pose_aggregator",
    srcs = ["pose_aggregator.cc"],
    hdrs = ["pose_aggregator.h"],
    linkstatic = 1,
    deps = [
        ":pose_bundle",
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "pose_bundle",
    srcs = ["pose_bundle.cc"],
    hdrs = ["pose_bundle.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
    ],
)

drake_cc_googletest(
    name = "pose_aggregator_test",
    linkstatic = 1,
    deps = [
        ":pose_aggregator",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
