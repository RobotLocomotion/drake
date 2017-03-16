# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary"
)

drake_cc_library(
    name = "particle",
    srcs = [
        "particle.cc",
    ],
    hdrs = [
        "particle.h",
        "particle-inl.h",
    ],
    deps = [
        "@eigen//:eigen",
        "//drake/common",
        "//drake/systems/framework:leaf_system"
    ]
)

drake_cc_library(
    name = "degenerate_euler_joint",
    srcs = [
	"degenerate_euler_joint.cc"
    ],
    hdrs = [
	"degenerate_euler_joint.h",
        "degenerate_euler_joint-inl.h"
    ],
    deps = [
        "@eigen//:eigen",
        "//drake/common",
        "//drake/systems/framework:leaf_system"
    ]
)

drake_cc_binary(
    name = "demo",
    srcs = [
        "uniformly_accelerated_particle.cc",
    ],
    data = [
        ":models"
    ],
    deps = [
        ":particle",
        ":degenerate_euler_joint",
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
        ":particle"
    ]
)

drake_cc_googletest(
    name = "degenerate_euler_joint_test",
    deps = [
        ":degenerate_euler_joint"
    ]
)

cpplint()
