# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "models/*.sdf",
    ]),
)

drake_cc_library(
    name = "particles",
    srcs = [
        "particle.cc",
        "utilities.cc",
    ],
    hdrs = [
        "particle.h",
        "utilities.h",
    ],
    deps = [
        "//drake/common",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/primitives:matrix_gain",
    ],
)

drake_cc_binary(
    name = "uniformly_accelerated_particle_demo",
    srcs = [
        "uniformly_accelerated_particle.cc",
    ],
    data = [
        ":models",
    ],
    deps = [
        ":particles",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "particle_test",
    deps = [
        ":particles",
    ],
)

drake_cc_googletest(
    name = "utilities_test",
    deps = [
        ":particles",
    ],
)

cpplint()
