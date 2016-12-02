# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "rotary_encoders_test",
    size = "small",
    srcs = ["rotary_encoders_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/sensors:rotary_encoders",
        "@gtest//:main",
    ],
)
