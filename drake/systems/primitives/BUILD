# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "primitives",
    srcs = [],
    hdrs = [],
    deps = [
        ":adder",
        ":affine_system",
        ":constant_value_source",
        ":constant_vector_source",
        ":demultiplexer",
        ":first_order_low_pass_filter",
        ":gain",
        ":integrator",
        ":linear_system",
        ":matrix_gain",
        ":multiplexer",
        ":pass_through",
        ":random_source",
        ":saturation",
        ":signal_log",
        ":signal_logger",
        ":trajectory_source",
        ":zero_order_hold",
    ],
)

drake_cc_library(
    name = "adder",
    srcs = ["adder.cc"],
    hdrs = ["adder.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "affine_system",
    srcs = ["affine_system.cc"],
    hdrs = ["affine_system.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "constant_value_source",
    srcs = ["constant_value_source.cc"],
    hdrs = [
        "constant_value_source.h",
        "constant_value_source-inl.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "constant_vector_source",
    srcs = ["constant_vector_source.cc"],
    hdrs = ["constant_vector_source.h"],
    deps = [
        "//drake/common:symbolic",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "demultiplexer",
    srcs = ["demultiplexer.cc"],
    hdrs = ["demultiplexer.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "first_order_low_pass_filter",
    srcs = ["first_order_low_pass_filter.cc"],
    hdrs = [
        "first_order_low_pass_filter.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "gain",
    srcs = ["gain.cc"],
    hdrs = [
        "gain.h",
        "gain-inl.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "integrator",
    srcs = ["integrator.cc"],
    hdrs = ["integrator.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "linear_system",
    srcs = ["linear_system.cc"],
    hdrs = ["linear_system.h"],
    deps = [
        ":affine_system",
        "//drake/math:autodiff",
        "//drake/math:gradient",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "matrix_gain",
    srcs = ["matrix_gain.cc"],
    hdrs = ["matrix_gain.h"],
    deps = [
        ":linear_system",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "multiplexer",
    srcs = ["multiplexer.cc"],
    hdrs = ["multiplexer.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "pass_through",
    srcs = ["pass_through.cc"],
    hdrs = [
        "pass_through.h",
        "pass_through-inl.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "random_source",
    srcs = ["random_source.cc"],
    hdrs = ["random_source.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "saturation",
    srcs = ["saturation.cc"],
    hdrs = ["saturation.h"],
    deps = [
        "//drake/math:saturate",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "signal_logger",
    srcs = ["signal_logger.cc"],
    hdrs = ["signal_logger.h"],
    deps = [
        ":signal_log",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "signal_log",
    srcs = ["signal_log.cc"],
    hdrs = ["signal_log.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "trajectory_source",
    srcs = ["trajectory_source.cc"],
    hdrs = ["trajectory_source.h"],
    deps = [
        "//drake/common/trajectories:trajectory",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "zero_order_hold",
    srcs = ["zero_order_hold.cc"],
    hdrs = [
        "zero_order_hold.h",
        "zero_order_hold-inl.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

drake_cc_library(
    name = "affine_linear_test",
    testonly = 1,
    hdrs = ["test/affine_linear_test.h"],
)

drake_cc_googletest(
    name = "adder_test",
    deps = [
        ":adder",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "affine_system_test",
    deps = [
        ":affine_linear_test",
        ":affine_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "constant_value_source_test",
    deps = [
        ":constant_value_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "constant_vector_source_test",
    deps = [
        ":constant_vector_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "demultiplexer_test",
    deps = [
        ":demultiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "first_order_low_pass_filter_test",
    srcs = [
        "test/first_order_low_pass_filter_test.cc",
    ],
    deps = [
        ":first_order_low_pass_filter",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
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

drake_cc_googletest(
    name = "integrator_test",
    deps = [
        ":integrator",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "linear_system_test",
    deps = [
        ":affine_linear_test",
        ":linear_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "matrix_gain_test",
    deps = [
        ":affine_linear_test",
        ":matrix_gain",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "multiplexer_test",
    deps = [
        ":multiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
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

drake_cc_googletest(
    name = "random_source_test",
    deps = [
        ":random_source",
        ":signal_logger",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

drake_cc_googletest(
    name = "saturation_test",
    deps = [
        ":saturation",
    ],
)

drake_cc_googletest(
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

drake_cc_googletest(
    name = "trajectory_source_test",
    deps = [
        ":trajectory_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "zero_order_hold_test",
    deps = [
        ":zero_order_hold",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cpplint()
