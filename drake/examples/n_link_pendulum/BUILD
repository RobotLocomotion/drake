# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "n_link_pendulum_plant",
    srcs = ["n_link_pendulum_plant.cc"],
    hdrs = ["n_link_pendulum_plant.h"],
    deps = [
        "//drake/geometry:geometry_ids",
        "//drake/geometry:geometry_system",
        "//drake/math:geometric_transform",
        "//drake/multibody/multibody_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

#drake_cc_binary(
#    name = "solar_system_run_dynamics",
#    srcs = ["solar_system_run_dynamics.cc"],
#    deps = [
#        ":solar_system",
#        "//drake/geometry:geometry_system",
#        "//drake/geometry:geometry_visualization",
#        "//drake/systems/analysis:simulator",
#        "//drake/systems/framework:diagram",
#        "//drake/systems/lcm",
#        "//drake/systems/primitives:constant_vector_source",
#        "//drake/systems/primitives:signal_logger",
#        "//drake/systems/rendering:pose_bundle_to_draw_message",
#    ],
#)

add_lint_tests()
