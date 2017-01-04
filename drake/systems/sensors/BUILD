# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensors",
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":rotary_encoders",
        ":depth_sensor",
    ],
)

cc_library(
    name = "rotary_encoders",
    srcs = ["rotary_encoders.cc"],
    hdrs = ["rotary_encoders.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "depth_sensor",
    srcs = ["depth_sensor.cc",
            "depth_sensor_output.cc",
            "depth_sensor_specification.cc"],
    hdrs = ["depth_sensor.h",
            "depth_sensor_output.h",
            "depth_sensor_specification.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
        "//drake/multibody:rigid_body_tree"
    ],
)

# === test/ ===

cc_googletest(
    name = "rotary_encoders_test",
    deps = [
        ":rotary_encoders",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "depth_sensor_test",
    srcs = ["test/depth_sensor_test.cc"],
    deps = [
        ":depth_sensor",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
