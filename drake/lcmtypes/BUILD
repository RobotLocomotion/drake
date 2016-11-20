# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])
load("//tools:lcm.bzl", "lcm_cc_library")

lcm_cc_library(
    name = "drake_signal",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
    linkstatic = 1)

lcm_cc_library(
    name = "automotive",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_driving_command_t.lcm",
        "lcmt_euler_floating_joint_state_t.lcm",
        "lcmt_simple_car_config_t.lcm",
        "lcmt_simple_car_state_t.lcm",
    ],
    linkstatic = 1)

lcm_cc_library(
    name = "iiwa",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_iiwa_command.lcm",
        "lcmt_iiwa_status.lcm",
    ],
    linkstatic = 1)

# TODO(jwnimmer-tri) Many more messages are missing ...
