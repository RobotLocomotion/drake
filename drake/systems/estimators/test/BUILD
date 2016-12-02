# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "luenberger_observer_test",
    size = "small",
    srcs = ["luenberger_observer_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/estimators:luenberger_observer",
        "//drake/systems/framework/primitives:linear_system",
        "@gtest//:main",
    ],
)
