# -*- python -*-

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

drake_cc_library(
    name = "particles",
    srcs = [
        "src/particle.cc",
	"src/degenerate_euler_joint.cc"
    ],
    hdrs = [
        "src/particle.hh",
        "src/particle-inl.hh",
    	"src/degenerate_euler_joint-inl.hh",
	"src/degenerate_euler_joint.hh"
    ],
    deps = [
        "@eigen//:eigen",
        "//drake/common",
        "//drake/systems/framework:leaf_system"
    ]
)

cc_binary(
    name = "uniformly_accelerated_particle",
    srcs = [
        "src/uniformly_accelerated_particle.cc",
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
        "//drake/multibody/parsers",
        "//drake/common:text_logging_gflags"
    ]
)

py_binary(
    name = "demo",
    srcs = ["src/demo.py"],
    data = [
        ":uniformly_accelerated_particle",
        "@drake_visualizer//:drake-visualizer"
    ],
    main = "src/demo.py",
    deps = [
        "@lcm//:lcm-python",
    ]
)
