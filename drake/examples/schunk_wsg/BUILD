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
    data = [
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":simulated_schunk_wsg_system",
        "//drake/lcm",
        "//drake/lcmtypes:schunk",
        "//drake/manipulation/schunk_wsg:schunk_wsg_constants",
        "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:matrix_gain",
        "//drake/systems/primitives:multiplexer",
        "@com_github_gflags_gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "schunk_wsg_lift_test",
    size = "large",
    data = [
        "test/test_lifter.sdf",
        "//drake/manipulation/models/wsg_50_description:models",
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

drake_cc_googletest(
    name = "simulated_schunk_wsg_system_test",
    data = [
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":simulated_schunk_wsg_system",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cpplint()
