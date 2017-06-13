# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "kuka_iiwa_robot",
    srcs = [
        "MGKukaIIwaRobot.cc",
        "kuka_iiwa_robot.cc",
    ],
    hdrs = [
        "MGKukaIIwaRobot.h",
        "kuka_iiwa_robot.h",
    ],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "kuka_iiwa_robot_test",
    size = "medium",
    deps = [
        ":kuka_iiwa_robot",
    ],
)

cpplint()
