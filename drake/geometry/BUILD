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
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "spatial_pose",
    srcs = [],
    hdrs = ["spatial_pose.h"],
    deps = ["//drake/common"],
)

# -----------------------------------------------------

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

drake_cc_googletest(
    name = "spatial_pose_test",
    deps = [
        ":spatial_pose",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
