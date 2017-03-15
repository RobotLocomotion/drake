# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "spatial_vector",
    srcs = [],
    hdrs = ["spatial_vector.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "spatial_velocity",
    srcs = [],
    hdrs = ["spatial_velocity.h"],
    deps = [
        ":spatial_vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "spatial_force",
    srcs = [],
    hdrs = ["spatial_force.h"],
    deps = [
        ":spatial_vector",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "spatial_algebra",
    srcs = [],
    hdrs = ["spatial_algebra.h"],
    deps = [
        ":spatial_force",
        ":spatial_vector",
        ":spatial_velocity",
        "//drake/common",
    ],
)

drake_cc_googletest(
    name = "spatial_algebra_tests",
    srcs = ["test/spatial_algebra_tests.cc"],
    deps = [
        ":spatial_algebra",
        "//drake/common:autodiff",
    ],
)
