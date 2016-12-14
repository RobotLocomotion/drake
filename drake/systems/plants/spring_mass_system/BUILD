# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spring_mass_system",
    srcs = ["spring_mass_system.cc"],
    hdrs = ["spring_mass_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

# === test/ ===

cc_test(
    name = "spring_mass_system_test",
    size = "small",
    srcs = ["test/spring_mass_system_test.cc"],
    linkstatic = 1,
    deps = [
        ":spring_mass_system",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)
