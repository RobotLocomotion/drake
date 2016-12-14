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
        "//drake/systems/primitives:linear_system",
    ],
)

cc_library(
    name = "luenberger_observer",
    srcs = ["luenberger_observer.cc"],
    hdrs = ["luenberger_observer.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["test/kalman_filter_test.cc"],
    linkstatic = 1,
    deps = [
        ":kalman_filter",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "luenberger_observer_test",
    size = "small",
    srcs = ["test/luenberger_observer_test.cc"],
    linkstatic = 1,
    deps = [
        ":luenberger_observer",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/primitives:linear_system",
        "@gtest//:main",
    ],
)
