# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary"
)

filegroup(
    name = "models",
    srcs = glob([
        "models/*.sdf",
    ]),
)

drake_cc_library(
    name = "sdf_helpers",
    srcs = [
        "sdf_helpers.cc"
    ],
    hdrs = [
        "sdf_helpers.h"
    ],
    deps = [
        "//drake/common",
        "//drake/multibody:rigid_body_tree",
        "@eigen",
        "@sdformat"
    ]
)

drake_cc_binary(
    name = "double_pendulum_demo",
    srcs = [
        "double_pendulum_demo.cc",
    ],
    data = [
        ":models",
    ],
    deps = [
        ":sdf_helpers",
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/framework",
        "@sdformat"
    ]
)

cpplint()
