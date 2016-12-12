# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "adder",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "affine_system",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "constant_value_source",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "constant_vector_source",
    deps = [
        "//drake/common:symbolic",
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "demultiplexer",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "gain",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "integrator",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "linear_system",
    deps = [
        ":affine_system",
        "//drake/math:autodiff",
        "//drake/math:gradient",
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "matrix_gain",
    deps = [
        ":linear_system",
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "multiplexer",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "pass_through",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "random_source",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "signal_logger",
    deps = [
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "trajectory_source",
    deps = [
        "//drake/common/trajectories:trajectory",
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "zero_order_hold",
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_library(
    name = "affine_linear_test",
    testonly = 1,
    hdrs = ["test/affine_linear_test.h"],
    linkstatic = 1,
)

cc_unit_test(
    name = "adder_test",
    deps = [
        ":adder",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "affine_system_test",
    deps = [
        ":affine_linear_test",
        ":affine_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "constant_value_source_test",
    deps = [
        ":constant_value_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "constant_vector_source_test",
    deps = [
        ":constant_vector_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "demultiplexer_test",
    deps = [
        ":demultiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "gain_test",
    srcs = [
        "test/gain_scalartype_test.cc",
        "test/gain_test.cc",
    ],
    deps = [
        ":gain",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:symbolic",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "integrator_test",
    deps = [
        ":integrator",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "linear_system_test",
    deps = [
        ":affine_linear_test",
        ":linear_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "matrix_gain_test",
    deps = [
        ":affine_linear_test",
        ":matrix_gain",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "multiplexer_test",
    deps = [
        ":multiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "pass_through_test",
    srcs = [
        "test/pass_through_scalartype_test.cc",
        "test/pass_through_test.cc",
    ],
    deps = [
        ":pass_through",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "random_source_test",
    deps = [
        ":random_source",
        ":signal_logger",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

cc_unit_test(
    name = "signal_logger_test",
    deps = [
        ":affine_system",
        ":linear_system",
        ":signal_logger",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "trajectory_source_test",
    deps = [
        ":trajectory_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/systems/framework",
    ],
)

cc_unit_test(
    name = "zero_order_hold_test",
    deps = [
        ":zero_order_hold",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)
