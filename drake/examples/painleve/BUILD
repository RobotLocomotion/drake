# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "painleve",
    srcs = ["painleve.cc"],
    hdrs = [
        "painleve.h",
        "painleve-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework:leaf_system",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "painleve_test",
    deps = [
        ":painleve",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

cpplint()
