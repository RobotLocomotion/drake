# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/common:eigen_matrix_compare",
    "//drake/systems/framework",
    "@gtest//:main",
]

cc_library(
    name = "affine_linear_test",
    testonly = 1,
    hdrs = ["affine_linear_test.h"],
    linkstatic = 1,
)

cc_test(
    name = "adder_test",
    size = "small",
    srcs = ["adder_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:adder",
    ],
)

cc_test(
    name = "affine_system_test",
    size = "small",
    srcs = ["affine_system_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:affine_system",
        ":affine_linear_test",
    ],
)

cc_test(
    name = "constant_value_source_test",
    size = "small",
    srcs = ["constant_value_source_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:constant_value_source",
    ],
)

cc_test(
    name = "constant_vector_source_test",
    size = "small",
    srcs = ["constant_vector_source_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:constant_vector_source",
    ],
)

cc_test(
    name = "demultiplexer_test",
    size = "small",
    srcs = ["demultiplexer_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:demultiplexer",
    ],
)

cc_test(
    name = "gain_test",
    size = "small",
    srcs = [
        "gain_scalartype_test.cc",
        "gain_test.cc",
    ],
    deps = DEPS + [
        "//drake/common:symbolic",
        "//drake/systems/framework/primitives:gain",
    ],
)

cc_test(
    name = "integrator_test",
    size = "small",
    srcs = ["integrator_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:integrator",
    ],
)

cc_test(
    name = "linear_system_test",
    size = "small",
    srcs = ["linear_system_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:linear_system",
        ":affine_linear_test",
    ],
)

cc_test(
    name = "matrix_gain_test",
    size = "small",
    srcs = ["matrix_gain_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:matrix_gain",
        ":affine_linear_test",
    ],
)

cc_test(
    name = "multiplexer_test",
    size = "small",
    srcs = ["multiplexer_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:multiplexer",
    ],
)

cc_test(
    name = "pass_through_test",
    size = "small",
    srcs = [
        "pass_through_scalartype_test.cc",
        "pass_through_test.cc",
    ],
    deps = DEPS + [
        "//drake/systems/framework/primitives:pass_through",
    ],
)

cc_test(
    name = "signal_logger_test",
    size = "small",
    srcs = ["signal_logger_test.cc"],
    deps = DEPS + [
        "//drake/systems/analysis",
        "//drake/systems/framework/primitives:affine_system",
        "//drake/systems/framework/primitives:linear_system",
        "//drake/systems/framework/primitives:signal_logger",
    ],
)

cc_test(
    name = "trajectory_source_test",
    size = "small",
    srcs = ["trajectory_source_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:trajectory_source",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
    ],
)

cc_test(
    name = "zero_order_hold_test",
    size = "small",
    srcs = ["zero_order_hold_test.cc"],
    deps = DEPS + [
        "//drake/systems/framework/primitives:zero_order_hold",
    ],
)
