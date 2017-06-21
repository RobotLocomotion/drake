# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)
load("//tools:mosek.bzl", "mosek_test_tags")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [
        ":luenberger_observer",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/primitives:linear_system",
    ],
)

drake_cc_library(
    name = "luenberger_observer",
    srcs = ["luenberger_observer.cc"],
    hdrs = ["luenberger_observer.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "kalman_filter_test",
    deps = [
        ":kalman_filter",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "luenberger_observer_test",
    deps = [
        ":luenberger_observer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/primitives:linear_system",
    ],
)

drake_cc_test(
    name = "pose_estimation_test",
    srcs = [
        "dev/pose_estimation_test.cc",
        "dev/rotation.h",
    ],
    data = ["//drake/multibody:models"],
    tags = mosek_test_tags(),
    deps = [
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/collision",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
        "@lcm",
        "@lcmtypes_bot2_core",
    ],
)

cpplint()
