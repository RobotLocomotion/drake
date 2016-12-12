# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "spring_mass_system",
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_unit_test(
    name = "spring_mass_system_test",
    deps = [
        ":spring_mass_system",
        "//drake/common:eigen_matrix_compare",
    ],
)
