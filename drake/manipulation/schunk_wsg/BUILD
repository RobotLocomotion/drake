# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "schunk_wsg_constants",
    hdrs = ["schunk_wsg_constants.h"],
    visibility = ["//visibility:public"],
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
    visibility = ["//visibility:public"],
    deps = [
        ":schunk_wsg_trajectory_generator_state_vector",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/common/trajectories:trajectory",
        "//drake/lcmtypes:schunk",
        "//drake/systems/framework:leaf_system",
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
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
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

cpplint()
