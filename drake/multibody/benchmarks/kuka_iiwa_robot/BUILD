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
    size = "small",
    deps = [
        ":kuka_iiwa_robot",
    ],
)

drake_cc_googletest(
    name = "kuka_iiwa_robot_kinematics_test",
    size = "small",
    deps = [
        ":kuka_iiwa_robot",
        "//drake/math:geometric_transform",
        "//drake/multibody/multibody_tree",
        "//drake/multibody/multibody_tree:bodies_and_frames",
        "//drake/multibody/multibody_tree:mobilizers_and_body_nodes",
    ],
)

cpplint()
