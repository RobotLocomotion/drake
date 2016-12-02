# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "direct_collocation_constraint_test",
    size = "small",
    srcs = ["direct_collocation_constraint_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/plants/constraint:direct_collocation_constraint",
        "@gtest//:main",
    ],
)
