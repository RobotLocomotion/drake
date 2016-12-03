# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gravity_compensator",
    srcs = ["gravity_compensator.cc"],
    hdrs = ["gravity_compensator.h"],
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "linear_optimal_control",
    srcs = ["linear_optimal_control.cc"],
    hdrs = ["linear_optimal_control.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:is_approx_equal_abstol",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:linear_system",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:pass_through",
    ],
)

cc_library(
    name = "pid_controlled_system",
    srcs = ["pid_controlled_system.cc"],
    hdrs = ["pid_controlled_system.h"],
    linkstatic = 1,
    deps = [
        ":pid_controller",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:demultiplexer",
        "//drake/systems/framework/primitives:matrix_gain",
    ],
)
