# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:common"]

cc_library(
    name = "trajectory",
    hdrs = ["trajectory.h"],
    deps = DEPS,
    linkstatic = 1)

cc_library(
    name = "piecewise_polynomial",
    srcs = [
        "exponential_plus_piecewise_polynomial.cc",
        "piecewise_function.cc",
        "piecewise_polynomial_base.cc",
        "piecewise_polynomial.cc",
    ],
    hdrs = [
        "exponential_plus_piecewise_polynomial.h",
        "piecewise_function.h",
        "piecewise_polynomial_base.h",
        "piecewise_polynomial.h",
    ],
    deps = DEPS + ["//drake/common:polynomial"],
    linkstatic = 1)

cc_library(
    name = "piecewise_polynomial_trajectory",
    srcs = ["piecewise_polynomial_trajectory.cc"],
    hdrs = ["piecewise_polynomial_trajectory.h"],
    deps = DEPS + [":trajectory", ":piecewise_polynomial"],
    linkstatic = 1)
