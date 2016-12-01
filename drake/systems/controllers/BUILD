# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
        "//drake/systems/framework/primitives:adder",
        "//drake/systems/framework/primitives:gain",
        "//drake/systems/framework/primitives:integrator",
        "//drake/systems/framework/primitives:pass_through",
    ],
)
