# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "controlled_spring_mass_system",
    testonly = 1,
    deps = [
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework",
        "//drake/systems/plants/spring_mass_system",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:gain",
    ],
)

cc_unit_test(
    name = "controlled_spring_mass_system_test",
    srcs = ["controlled_spring_mass_system_test.cc"],
    linkstatic = 1,
    deps = [
        ":controlled_spring_mass_system",
    ],
)
