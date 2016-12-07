# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "my_spring_mass_system",
    testonly = 1,
    hdrs = ["my_spring_mass_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/plants/spring_mass_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "simulator_test",
    size = "small",
    srcs = ["simulator_test.cc"],
    linkstatic = 1,
    deps = [
        ":my_spring_mass_system",
        "//drake/systems/analysis",
        "//drake/systems/analysis/test/controlled_spring_mass_system",
        "@gtest//:main",
    ],
)

cc_test(
    name = "explicit_euler_integrator_test",
    size = "small",
    srcs = ["explicit_euler_integrator_test.cc"],
    linkstatic = 1,
    deps = [
        ":my_spring_mass_system",
        "//drake/systems/analysis",
        "@gtest//:main",
    ],
)

cc_test(
    name = "runge_kutta2_integrator_test",
    size = "small",
    srcs = ["runge_kutta2_integrator_test.cc"],
    linkstatic = 1,
    deps = [
        ":my_spring_mass_system",
        "//drake/systems/analysis",
        "@gtest//:main",
    ],
)
