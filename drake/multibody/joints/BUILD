# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "joints",
    srcs = [
        "drake_joint.cc",
        "fixed_joint.cc",
        "helical_joint.cc",
        "prismatic_joint.cc",
        "quaternion_floating_joint.cc",
        "revolute_joint.cc",
        "roll_pitch_yaw_floating_joint.cc",
    ],
    hdrs = [
        "drake_joint.h",
        "drake_joint_impl.h",
        "drake_joints.h",
        "fixed_axis_one_dof_joint.h",
        "fixed_joint.h",
        "floating_base_types.h",
        "helical_joint.h",
        "prismatic_joint.h",
        "quaternion_floating_joint.h",
        "revolute_joint.h",
        "roll_pitch_yaw_floating_joint.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/util",
    ],
)

cc_googletest(
    name = "joint_test",
    deps = [
        ":joints",
    ],
)
