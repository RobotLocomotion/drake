# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "body_motion_data",
    srcs = ["BodyMotionData.cpp"],
    hdrs = ["BodyMotionData.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

drake_cc_library(
    name = "qp_locomotion_plan",
    srcs = ["QPLocomotionPlan.cpp"],
    hdrs = ["QPLocomotionPlan.h"],
    deps = [
        ":body_motion_data",
        ":side",
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/common/trajectories/qp_spline:spline_generation",
        "//drake/examples/Atlas:atlas_util",
        "//drake/lcmtypes:qp_controller_input",
        "//drake/math:autodiff",
        "//drake/math:expmap",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/controllers:zmp_util",
        "//drake/util",
        "//drake/util:convex_hull",
        "//drake/util:lcm_util",
    ],
)

drake_cc_library(
    name = "side",
    srcs = ["Side.cpp"],
    hdrs = ["Side.h"],
    deps = [],
)

cpplint()
