# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "acrobot_plant",
    srcs = ["acrobot_plant.cc"],
    hdrs = ["acrobot_plant.h"],
    deps = [
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/multibody/parsers",
        "//drake/systems/sensors:rotary_encoders",
        "//drake/systems/framework",      
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up",
    srcs = ["acrobot_run_swing_up.cc"],
    deps = [
        "//drake/examples/Acrobot:acrobot_plant",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
    ],
)


cpplint()
