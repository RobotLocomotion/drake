# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("//tools:lcm.bzl", "lcm_cc_library", "lcm_java_library", "lcm_py_library")

lcm_cc_library(
    name = "drake_signal",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
    linkstatic = 1,
)

lcm_py_library(
    name = "drake_signal-py",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
)

_VIEWER_LCM_SRCS = [
    "lcmt_viewer_command.lcm",
    "lcmt_viewer_draw.lcm",
    "lcmt_viewer_geometry_data.lcm",
    "lcmt_viewer_link_data.lcm",
    "lcmt_viewer_load_robot.lcm",
]

lcm_cc_library(
    name = "viewer",
    lcm_package = "drake",
    lcm_srcs = _VIEWER_LCM_SRCS,
    linkstatic = 1,
)

lcm_java_library(
    name = "viewer-java",
    lcm_package = "drake",
    lcm_srcs = _VIEWER_LCM_SRCS,
)

_AUTOMOTIVE_LCM_SRCS = [
    "lcmt_driving_command_t.lcm",
    "lcmt_endless_road_car_config_t.lcm",
    "lcmt_endless_road_car_state_t.lcm",
    "lcmt_endless_road_oracle_output_t.lcm",
    "lcmt_euler_floating_joint_state_t.lcm",
    "lcmt_simple_car_config_t.lcm",
    "lcmt_simple_car_state_t.lcm",
]

lcm_cc_library(
    name = "automotive",
    lcm_package = "drake",
    lcm_srcs = _AUTOMOTIVE_LCM_SRCS,
    linkstatic = 1,
)

lcm_java_library(
    name = "automotive-java",
    lcm_package = "drake",
    lcm_srcs = _AUTOMOTIVE_LCM_SRCS,
)

lcm_py_library(
    name = "automotive-py",
    lcm_package = "drake",
    lcm_srcs = _AUTOMOTIVE_LCM_SRCS,
)

lcm_cc_library(
    name = "iiwa",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_iiwa_command.lcm",
        "lcmt_iiwa_status.lcm",
    ],
    linkstatic = 1,
)

lcm_cc_library(
    name = "call_matlab",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_call_matlab.lcm",
        "lcmt_matlab_array.lcm",
    ],
    linkstatic = 1,
)

# TODO(jwnimmer-tri) Many more messages are missing ...
