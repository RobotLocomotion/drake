# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adder",
    hdrs = ["adder.h"],
    srcs = ["adder.cc"],
    deps = [
        "//drake/systems/framework",
    ],
    linkstatic=1)

cc_library(
    name = "constant_vector_source",
    hdrs = ["constant_vector_source.h"],
    srcs = ["constant_vector_source.cc"],
    deps = [
        "//drake/systems/framework",
        "//drake/common:symbolic",
    ],
    linkstatic=1)

cc_library(
    name = "gain",
    hdrs = ["gain.h",
            "gain-inl.h"],
    srcs = ["gain.cc"],
    deps = [
        "//drake/systems/framework",
    ],
    linkstatic=1)

cc_library(
    name = "integrator",
    hdrs = ["integrator.h"],
    srcs = ["integrator.cc"],
    deps = [
        "//drake/systems/framework",
    ],
    linkstatic=1)

cc_library(
    name = "zero_order_hold",
    hdrs = ["zero_order_hold.h",
            "zero_order_hold-inl.h"],
    srcs = ["zero_order_hold.cc"],
    deps = [
        "//drake/systems/framework",
    ],
    linkstatic=1)
