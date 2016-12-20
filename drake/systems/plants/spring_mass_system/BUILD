# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

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

cc_googletest(
    name = "spring_mass_system_test",
    deps = [
        ":spring_mass_system",
        "//drake/common:eigen_matrix_compare",
    ],
)
