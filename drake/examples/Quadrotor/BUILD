# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "quadrotor_plant",
    srcs = ["quadrotor_plant.cc"],
    hdrs = ["quadrotor_plant.h"],
    deps = [
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/primitives:affine_system",
        "//drake/util",
    ],
)

drake_cc_binary(
    name = "run_quadrotor_dynamics",
    srcs = ["run_quadrotor_dynamics.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--duration=1.0"],
    test_rule_size = "medium",
    deps = [
        ":quadrotor_plant",
        "//drake/common:drake_path",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
        "@gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "run_quadrotor_lqr",
    srcs = ["run_quadrotor_lqr.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = [
        "-simulation_trials=2",
        "-simulation_real_time_rate=0.0",
    ],
    deps = [
        ":quadrotor_plant",
        "//drake/common:drake_path",
        "//drake/common:is_approx_equal_abstol",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "@gflags//:gflags",
    ],
)

drake_cc_googletest(
    name = "quadrotor_dynamics_test",
    data = [":models"],
    deps = [
        ":quadrotor_plant",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cpplint()
