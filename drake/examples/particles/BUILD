# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary"
)

drake_cc_library(
    name = "particles",
    srcs = [
        "particle.cc",
        "degenerate_euler_joint.cc"
    ],
    hdrs = [
        "particle.h",
        "degenerate_euler_joint.h"
    ],
    deps = [
        "//drake/common",
        "//drake/systems/framework"
    ]
)

drake_cc_binary(
    name = "uniformly_accelerated_particle_demo",
    srcs = [
        "uniformly_accelerated_particle.cc",
    ],
    data = [
        ":models"
    ],
    deps = [
        ":particles",
        "//drake/lcm",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/common:text_logging_gflags",
        "//drake/multibody/parsers"
    ]
)

# === test/ ===

drake_cc_googletest(
    name = "particle_test",
    deps = [
        ":particles"
    ]
)

drake_cc_googletest(
    name = "degenerate_euler_joint_test",
    deps = [
        ":particles"
    ]
)

cpplint()
