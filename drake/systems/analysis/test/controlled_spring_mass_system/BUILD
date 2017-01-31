# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "controlled_spring_mass_system",
    testonly = 1,
    srcs = ["controlled_spring_mass_system.cc"],
    hdrs = ["controlled_spring_mass_system.h"],
    linkstatic = 1,
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

drake_cc_googletest(
    name = "controlled_spring_mass_system_test",
    srcs = ["controlled_spring_mass_system_test.cc"],
    deps = [
        ":controlled_spring_mass_system",
    ],
)

cpplint()
