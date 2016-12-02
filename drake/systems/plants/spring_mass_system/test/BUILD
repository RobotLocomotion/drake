# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "spring_mass_system_test",
    size = "small",
    srcs = ["spring_mass_system_test.cc"],
    linkstatic = 1,
    deps = [
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/plants/spring_mass_system",
        "@gtest//:main",
    ],
)
