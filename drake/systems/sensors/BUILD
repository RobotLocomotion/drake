# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "sensors",
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":accelerometer",
        ":depth_sensor",
        ":rotary_encoders",
    ],
)

drake_cc_library(
    name = "rotary_encoders",
    srcs = ["rotary_encoders.cc"],
    hdrs = ["rotary_encoders.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "accelerometer",
    srcs = [
        "accelerometer.cc",
        "accelerometer_output.cc",
    ],
    hdrs = [
        "accelerometer.h",
        "accelerometer_output.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "depth_sensor",
    srcs = [
        "depth_sensor.cc",
        "depth_sensor_output.cc",
        "depth_sensor_specification.cc",
    ],
    hdrs = [
        "depth_sensor.h",
        "depth_sensor_output.h",
        "depth_sensor_specification.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "accelerometer_example_diagram",
    srcs = [
        "test/accelerometer_test/accelerometer_example_diagram.cc",
        "test/accelerometer_test/accelerometer_test_logger.cc",
        "test/accelerometer_test/accelerometer_xdot_filter.cc",
    ],
    hdrs = [
        "test/accelerometer_test/accelerometer_example_diagram.h",
        "test/accelerometer_test/accelerometer_test_logger.h",
        "test/accelerometer_test/accelerometer_xdot_filter.h",
    ],
    data = [
        "//drake/examples/Pendulum:models",
    ],
    linkstatic = 1,
    deps = [
        ":accelerometer",
        "//drake/common",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_that_publishes_xdot",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/primitives:constant_vector_source",
    ],
)


drake_cc_binary(
    name = "accelerometer_example",
    srcs = [
        "test/accelerometer_test/accelerometer_example.cc",
    ],
    deps = [
        ":accelerometer_example_diagram",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "rotary_encoders_test",
    deps = [
        ":rotary_encoders",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "accelerometer_test",
    srcs = [
        "test/accelerometer_test/accelerometer_test.cc",
        "test/accelerometer_test/accelerometer_example_diagram.cc",
        "test/accelerometer_test/accelerometer_example_diagram.h",
    ],
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":accelerometer_example_diagram",
        "//drake/lcm:mock",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/analysis:simulator",
    ],
)

drake_cc_googletest(
    name = "depth_sensor_test",
    srcs = ["test/depth_sensor_test.cc"],
    deps = [
        ":depth_sensor",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
