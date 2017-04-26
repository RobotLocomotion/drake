# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "sensors",
    srcs = [],
    hdrs = [],
    deps = [
        ":accelerometer",
        ":camera_info",
        ":depth_sensor",
        ":global_collision_sensor",
        ":rotary_encoders",
    ],
)

drake_cc_library(
    name = "camera_info",
    srcs = [
        "camera_info.cc",
    ],
    hdrs = [
        "camera_info.h",
    ],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "global_collision_sensor",
    srcs = [
        "global_collision_sensor.cc",
    ],
    hdrs = [
        "global_collision_sensor.h",
    ],
    deps = [
        "//drake/systems/framework",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_library(
    name = "image",
    srcs = [],
    hdrs = ["image.h"],
    deps = [
        "//drake/common",
        "//drake/common:reinit_after_move",
    ],
)

drake_cc_library(
    name = "rotary_encoders",
    srcs = ["rotary_encoders.cc"],
    hdrs = ["rotary_encoders.h"],
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
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
        "//drake/systems/rendering:pose_vector",
    ],
)

drake_cc_library(
    name = "depth_sensor_to_lcm_point_cloud_message",
    srcs = [
        "depth_sensor_to_lcm_point_cloud_message.cc",
    ],
    hdrs = [
        "depth_sensor_to_lcm_point_cloud_message.h",
    ],
    deps = [
        ":depth_sensor",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/rendering:pose_vector",
        "@bot_core_lcmtypes//:lib",
    ],
)

drake_cc_binary(
    name = "depth_sensor_to_lcm_point_cloud_message_demo",
    srcs = [
        "depth_sensor_to_lcm_point_cloud_message_demo.cc",
    ],
    deps = [
        ":depth_sensor_to_lcm_point_cloud_message",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
    ],
)

drake_cc_library(
    name = "gyroscope",
    srcs = [
        "gyroscope.cc",
        "gyroscope_output.cc",
    ],
    hdrs = [
        "gyroscope.h",
        "gyroscope_output.h",
    ],
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
        "test/accelerometer_test/accelerometer_xdot_hack.cc",
    ],
    hdrs = [
        "test/accelerometer_test/accelerometer_example_diagram.h",
        "test/accelerometer_test/accelerometer_test_logger.h",
        "test/accelerometer_test/accelerometer_xdot_hack.h",
    ],
    data = [
        "//drake/examples/Pendulum:models",
    ],
    deps = [
        ":accelerometer",
        "//drake/common",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_that_publishes_xdot",
        "//drake/systems/lcm:lcmt_drake_signal_translator",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:signal_logger",
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
    name = "camera_info_test",
    deps = [
        ":camera_info",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "global_collision_sensor_test",
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":global_collision_sensor",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "image_test",
    deps = [
        ":image",
    ],
)

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
    ],
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":accelerometer_example_diagram",
        "//drake/common:eigen_matrix_compare",
        "//drake/lcm:mock",
        "//drake/systems/analysis:simulator",
    ],
)

drake_cc_googletest(
    name = "depth_sensor_test",
    srcs = ["test/depth_sensor_test.cc"],
    deps = [
        ":depth_sensor",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "depth_sensor_to_lcm_point_cloud_message_test",
    srcs = ["test/depth_sensor_to_lcm_point_cloud_message_test.cc"],
    deps = [
        ":depth_sensor_to_lcm_point_cloud_message",
        "//drake/common:drake_path",
        "//drake/lcm:mock",
    ],
)

drake_cc_googletest(
    name = "gyroscope_test",
    srcs = ["test/gyroscope_test.cc"],
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":gyroscope",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

cpplint()
