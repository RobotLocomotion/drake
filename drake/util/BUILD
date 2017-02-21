# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "util",
    srcs = [
        "drakeGeometryUtil.cpp",
        "drakeUtil.cpp",
    ],
    hdrs = [
        "drakeGeometryUtil.h",
        "drakeUtil.h",
    ],
    deprecation = "Consider pitching in by porting the dependencies you need to //drake/math.",
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
    ],
)

drake_cc_library(
    name = "lcm_util",
    srcs = ["lcmUtil.cpp"],
    hdrs = ["lcmUtil.h"],
    deps = [
        ":util",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/lcmtypes:piecewise_polynomial",
        "//drake/lcmtypes:polynomial",
        "//drake/lcmtypes:polynomial_matrix",
        "//drake/lcmtypes:qp_controller_input",
        "//drake/math:geometric_transform",
        "@bot_core_lcmtypes//:lib",
        "@eigen//:eigen",
    ],
)

drake_cc_library(
    name = "app_util",
    hdrs = ["drakeAppUtil.h"],
    deprecation = "Please use gflags instead of drakeAppUtil.h.",
)

cpplint()
