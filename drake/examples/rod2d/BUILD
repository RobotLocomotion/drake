# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rod2d",
    srcs = ["rod2d.cc"],
    hdrs = [
        "rod2d.h",
        "rod2d-inl.h",
    ],
    deps = [
        "//drake/common",
        "//drake/solvers:mathematical_program",
        "//drake/systems/analysis",
        "//drake/systems/framework:leaf_system",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "rod2d_test",
    deps = [
        ":rod2d",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

cpplint()
