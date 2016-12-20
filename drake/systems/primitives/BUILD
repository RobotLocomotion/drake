# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adder",
    srcs = ["adder.cc"],
    hdrs = ["adder.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "affine_system",
    srcs = ["affine_system.cc"],
    hdrs = ["affine_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "constant_value_source",
    srcs = ["constant_value_source.cc"],
    hdrs = [
        "constant_value_source.h",
        "constant_value_source-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "constant_vector_source",
    srcs = ["constant_vector_source.cc"],
    hdrs = ["constant_vector_source.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:symbolic",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "demultiplexer",
    srcs = ["demultiplexer.cc"],
    hdrs = ["demultiplexer.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "gain",
    srcs = ["gain.cc"],
    hdrs = [
        "gain.h",
        "gain-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "integrator",
    srcs = ["integrator.cc"],
    hdrs = ["integrator.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "linear_system",
    srcs = ["linear_system.cc"],
    hdrs = ["linear_system.h"],
    linkstatic = 1,
    deps = [
        ":affine_system",
        "//drake/math:autodiff",
        "//drake/math:gradient",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "matrix_gain",
    srcs = ["matrix_gain.cc"],
    hdrs = ["matrix_gain.h"],
    linkstatic = 1,
    deps = [
        ":linear_system",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "multiplexer",
    srcs = ["multiplexer.cc"],
    hdrs = ["multiplexer.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "pass_through",
    srcs = ["pass_through.cc"],
    hdrs = [
        "pass_through.h",
        "pass_through-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "random_source",
    srcs = ["random_source.cc"],
    hdrs = ["random_source.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "signal_logger",
    srcs = ["signal_logger.cc"],
    hdrs = ["signal_logger.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "trajectory_source",
    srcs = ["trajectory_source.cc"],
    hdrs = ["trajectory_source.h"],
    linkstatic = 1,
    deps = [
        "//drake/common/trajectories:trajectory",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "zero_order_hold",
    srcs = ["zero_order_hold.cc"],
    hdrs = [
        "zero_order_hold.h",
        "zero_order_hold-inl.h",
    ],
    linkstatic = 1,
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

cc_googletest(
    name = "adder_test",
    deps = [
        ":adder",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "affine_system_test",
    deps = [
        ":affine_linear_test",
        ":affine_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "constant_value_source_test",
    deps = [
        ":constant_value_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "constant_vector_source_test",
    deps = [
        ":constant_vector_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "demultiplexer_test",
    deps = [
        ":demultiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
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

cc_googletest(
    name = "integrator_test",
    deps = [
        ":integrator",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "linear_system_test",
    deps = [
        ":affine_linear_test",
        ":linear_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "matrix_gain_test",
    deps = [
        ":affine_linear_test",
        ":matrix_gain",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "multiplexer_test",
    deps = [
        ":multiplexer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

cc_googletest(
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

cc_googletest(
    name = "random_source_test",
    deps = [
        ":random_source",
        ":signal_logger",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis",
    ],
)

cc_googletest(
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

cc_googletest(
    name = "trajectory_source_test",
    deps = [
        ":trajectory_source",
        "//drake/common:eigen_matrix_compare",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/systems/framework",
    ],
)

cc_googletest(
    name = "zero_order_hold_test",
    deps = [
        ":zero_order_hold",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)
