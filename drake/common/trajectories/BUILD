# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:common"]

cc_library(
    name = "trajectory",
    hdrs = ["trajectory.h"],
    linkstatic = 1,
    deps = DEPS,
)

cc_library(
    name = "piecewise_polynomial",
    srcs = [
        "exponential_plus_piecewise_polynomial.cc",
        "piecewise_function.cc",
        "piecewise_polynomial.cc",
        "piecewise_polynomial_base.cc",
    ],
    hdrs = [
        "exponential_plus_piecewise_polynomial.h",
        "piecewise_function.h",
        "piecewise_polynomial.h",
        "piecewise_polynomial_base.h",
    ],
    linkstatic = 1,
    deps = DEPS + ["//drake/common:polynomial"],
)

cc_library(
    name = "piecewise_polynomial_trajectory",
    srcs = ["piecewise_polynomial_trajectory.cc"],
    hdrs = ["piecewise_polynomial_trajectory.h"],
    linkstatic = 1,
    deps = DEPS + [
        ":trajectory",
        ":piecewise_polynomial",
    ],
)
