# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "continuity_constraint",
    deps = [
        "//drake/common",
    ],
)

cc_component(
    name = "spline_generation",
    deps = [
        ":spline_information",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

cc_component(
    name = "spline_information",
    deps = [
        ":continuity_constraint",
        ":value_constraint",
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

cc_component(
    name = "value_constraint",
    deps = [
        "//drake/common",
    ],
)

# === test/ ===

cc_unit_test(
    name = "spline_generation_test",
    deps = [
        ":spline_generation",
    ],
)
