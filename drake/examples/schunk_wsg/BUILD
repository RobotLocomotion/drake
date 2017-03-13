# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "schunk_wsg_constants",
    hdrs = ["schunk_wsg_constants.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "schunk_wsg_trajectory_generator_state_vector",
    srcs = ["gen/schunk_wsg_trajectory_generator_state_vector.cc"],
    hdrs = ["gen/schunk_wsg_trajectory_generator_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "schunk_wsg_lcm",
    srcs = ["schunk_wsg_lcm.cc"],
    hdrs = ["schunk_wsg_lcm.h"],
    visibility = ["//drake/examples/kuka_iiwa_arm:__subpackages__"],
    deps = [
        ":schunk_wsg_trajectory_generator_state_vector",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcmtypes:schunk",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "simulated_schunk_wsg_system",
    srcs = ["simulated_schunk_wsg_system.cc"],
    hdrs = ["simulated_schunk_wsg_system.h"],
    deps = [
        "//drake/common:drake_path",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_binary(
    name = "schunk_wsg_simulation",
    srcs = ["schunk_wsg_simulation.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":schunk_wsg_constants",
        ":schunk_wsg_lcm",
        ":simulated_schunk_wsg_system",
        "//drake/lcm",
        "//drake/lcmtypes:schunk",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:matrix_gain",
        "//drake/systems/primitives:multiplexer",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "schunk_wsg_constants_test",
    data = [
        ":models",
    ],
    deps = [
        ":schunk_wsg_constants",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "schunk_wsg_lift_test",
    size = "large",
    data = [
        ":models",
        "//drake/multibody:models",
    ],
    # This test is prohibitively slow with --compilation_mode=dbg.
    disable_in_compilation_mode_dbg = True,
    deps = [
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:contact_results_to_lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:trajectory_source",
    ],
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

cpplint()
