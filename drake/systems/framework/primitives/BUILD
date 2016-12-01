# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adder",
    srcs = ["adder.cc"],
    hdrs = ["adder.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "affine_system",
    srcs = ["affine_system.cc"],
    hdrs = ["affine_system.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)


cc_library(
    name = "constant_vector_source",
    srcs = ["constant_vector_source.cc"],
    hdrs = ["constant_vector_source.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:symbolic",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "gain",
    srcs = ["gain.cc"],
    hdrs = [
        "gain.h",
        "gain-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "integrator",
    srcs = ["integrator.cc"],
    hdrs = ["integrator.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "linear_system",
    srcs = ["linear_system.cc"],
    hdrs = ["linear_system.h"],
    linkstatic = 1,
    deps = [
        ":affine_system",
        "//drake/math:autodiff",
        "//drake/math:gradient",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "multiplexer",
    srcs = ["multiplexer.cc"],
    hdrs = ["multiplexer.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "signal_logger",
    srcs = ["signal_logger.cc"],
    hdrs = ["signal_logger.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)


cc_library(
    name = "zero_order_hold",
    srcs = ["zero_order_hold.cc"],
    hdrs = [
        "zero_order_hold.h",
        "zero_order_hold-inl.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)
