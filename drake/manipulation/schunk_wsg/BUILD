# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "schunk_wsg_constants",
    hdrs = ["schunk_wsg_constants.h"],
    deps = [
        "//drake/common",
    ],
)

drake_cc_library(
    name = "schunk_wsg_trajectory_generator_state_vector",
    srcs = ["gen/schunk_wsg_trajectory_generator_state_vector.cc"],
    hdrs = ["gen/schunk_wsg_trajectory_generator_state_vector.h"],
    visibility = [":__subpackages__"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "schunk_wsg_lcm",
    srcs = ["schunk_wsg_lcm.cc"],
    hdrs = ["schunk_wsg_lcm.h"],
    deps = [
        ":schunk_wsg_trajectory_generator_state_vector",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcmtypes:schunk",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "schunk_wsg_controller",
    srcs = ["schunk_wsg_controller.cc"],
    hdrs = ["schunk_wsg_controller.h"],
    deps = [
        ":schunk_wsg_constants",
        ":schunk_wsg_lcm",
        "//drake/common",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:diagram_builder",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:pass_through",
        "//drake/systems/primitives:saturation",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "schunk_wsg_constants_test",
    data = [
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":schunk_wsg_constants",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "schunk_wsg_controller_test",
    deps = [
        ":schunk_wsg_constants",
        ":schunk_wsg_controller",
        "//drake/systems/analysis:simulator",
    ],
)

drake_cc_googletest(
    name = "schunk_wsg_lcm_test",
    data = [
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    deps = [
        ":schunk_wsg_lcm",
        "//drake/systems/analysis:simulator",
    ],
)

add_lint_tests()
