# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "Acrobot.urdf",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "acrobot_state_vector",
    srcs = ["gen/acrobot_state_vector.cc"],
    hdrs = ["gen/acrobot_state_vector.h"],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "acrobot_plant",
    srcs = ["acrobot_plant.cc"],
    hdrs = ["acrobot_plant.h"],
    deps = [
        ":acrobot_state_vector",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/multibody/parsers",
        "//drake/systems/sensors:rotary_encoders",
        "//drake/systems/framework",      
    ],
)

drake_cc_binary(
    name = "acrobot_run_swing_up",
    srcs = ["acrobot_run_swing_up.cc",
            "acrobot_spong_controller.h"
    ],
    deps = [
        ":acrobot_plant",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/lcm:lcm",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "@gflags//:gflags",
    ],
)


cpplint()
