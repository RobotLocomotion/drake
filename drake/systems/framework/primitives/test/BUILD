# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = [
    "//drake/systems/framework",
    "@gtest//:main",
]

cc_test(
    name = "signal_logger_test",
    size = "small",
    srcs = ["signal_logger_test.cc"],
    deps = DEPS + [
        "//drake/systems/analysis",
        "//drake/systems/framework/primitives:affine_system",
        "//drake/systems/framework/primitives:linear_system",
        "//drake/systems/framework/primitives:signal_logger",
        "//drake/common:eigen_matrix_compare",
    ],
)
