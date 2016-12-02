# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controlled_spring_mass_system",
    testonly = 1,
    srcs = ["controlled_spring_mass_system.cc"],
    hdrs = ["controlled_spring_mass_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:constant_vector_source",
        "//drake/systems/framework/primitives:demultiplexer",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/plants/spring_mass_system",
    ],
)

cc_test(
    name = "controlled_spring_mass_system_test",
    size = "small",
    srcs = ["controlled_spring_mass_system_test.cc"],
    linkstatic = 1,
    deps = [
        ":controlled_spring_mass_system",
        "@gtest//:main",
    ],
)
