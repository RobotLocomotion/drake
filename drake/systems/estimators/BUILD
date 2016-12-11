# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    linkstatic = 1,
    deps = [
        ":luenberger_observer",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:linear_system",
    ],
)

cc_library(
    name = "luenberger_observer",
    srcs = ["luenberger_observer.cc"],
    hdrs = ["luenberger_observer.h"],
    linkstatic = 1,
    deps = ["//drake/systems/framework"],
)
