# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "rotary_encoders",
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_unit_test(
    name = "rotary_encoders_test",
    deps = [
        ":rotary_encoders",
        "//drake/common:eigen_matrix_compare",
    ],
)
