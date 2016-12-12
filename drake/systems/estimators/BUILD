# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "kalman_filter",
    deps = [
        ":luenberger_observer",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/primitives:linear_system",
    ],
)

cc_component(
    name = "luenberger_observer",
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_unit_test(
    name = "kalman_filter_test",
    deps = [
        ":kalman_filter",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "luenberger_observer_test",
    deps = [
        ":luenberger_observer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/primitives:linear_system",
    ],
)
