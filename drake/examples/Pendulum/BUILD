# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "cc_googletest")

cc_library(
    name = "pendulum_state_vector",
    srcs = ["gen/pendulum_state_vector.cc"],
    hdrs = ["gen/pendulum_state_vector.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework:vector",
    ],
)

cc_library(
    name = "pendulum_plant",
    srcs = ["pendulum_plant.cc"],
    hdrs = ["pendulum_plant.h"],
    linkstatic = 1,
    deps = [
        ":pendulum_state_vector",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "pendulum_swing_up",
    srcs = ["pendulum_swing_up.cc"],
    hdrs = ["pendulum_swing_up.h"],
    deps = [
        ":pendulum_plant",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

# === test/ ===

cc_googletest(
    name = "pendulum_urdf_dynamics_test",
    data = ["Pendulum.urdf"],
    deps = [
        ":pendulum_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

cc_googletest(
    name = "pendulum_dynamic_constraint_test",
    deps = [
        ":pendulum_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/trajectory_optimization:direct_collocation",
    ],
)

cc_googletest(
    name = "pendulum_trajectory_optimization_test",
    deps = [
        ":pendulum_swing_up",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
