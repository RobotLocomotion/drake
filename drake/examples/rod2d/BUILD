# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_binary(
    name = "rod2d_sim",
    srcs = ["rod2d_sim.cc"],
    add_test_rule = 1,
    test_rule_args = [" --sim_duration=0.01"],
    deps = [
        ":rod2d",
        "//drake/common",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_aggregator",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_library(
    name = "rod2d",
    srcs = ["rod2d.cc"],
    hdrs = [
        "rod2d.h",
        "rod2d-inl.h",
    ],
    deps = [
        "//drake/common",
        "//drake/multibody/rigid_constraint",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/rendering:pose_vector",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "rod2d_test",
    deps = [
        ":rod2d",
        "//drake/common",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/rigid_constraint:rigid_constraint_solver",
        "//drake/systems/analysis",
    ],
)

add_lint_tests()
