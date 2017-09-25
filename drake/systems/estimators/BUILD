# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_test",
)
load("//tools:lint.bzl", "add_lint_tests")
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
        "//drake/common/test_utilities:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "luenberger_observer_test",
    deps = [
        ":luenberger_observer",
        "//drake/common/test_utilities:eigen_matrix_compare",
        "//drake/common/test_utilities:is_dynamic_castable",
        "//drake/examples/pendulum:pendulum_plant",
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
        "//drake/common:essential",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/collision",
        "//drake/multibody/parsers",
        "//drake/solvers:mathematical_program",
        "@lcm",
        "@lcmtypes_bot2_core",
    ],
)

add_lint_tests()
