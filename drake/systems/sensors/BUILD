# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rotary_encoders",
    srcs = ["rotary_encoders.cc"],
    hdrs = ["rotary_encoders.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_test(
    name = "rotary_encoders_test",
    size = "small",
    srcs = ["test/rotary_encoders_test.cc"],
    linkstatic = 1,
    deps = [
        ":rotary_encoders",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)
