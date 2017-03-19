# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

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
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
    ],
)

drake_cc_library(
    name = "convex_hull",
    srcs = ["convexHull.cpp"],
    hdrs = ["convexHull.h"],
    deps = [
        "//drake/common",
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
    name = "yaml_util",
    srcs = ["yaml/yamlUtil.cpp"],
    hdrs = ["yaml/yamlUtil.h"],
    deps = [
        "//drake/common",
        "//drake/systems/controllers:qp_common",
        "@yaml_cpp//:lib",
    ],
)

drake_cc_library(
    name = "app_util",
    hdrs = ["drakeAppUtil.h"],
    deprecation = "Please use gflags instead of drakeAppUtil.h.",
)

# === test/ ===

drake_cc_googletest(
    name = "testConvexHull",
    srcs = ["test/testConvexHull.cpp"],
    deps = [
        ":convex_hull",
    ],
)

drake_cc_googletest(
    name = "testDrakeGeometryUtil",
    srcs = ["test/testDrakeGeometryUtil.cpp"],
    deps = [
        ":util",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "testLCMUtil",
    srcs = ["test/testLCMUtil.cpp"],
    deps = [
        ":lcm_util",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:random_polynomial_matrix",
        "//drake/common/trajectories:random_piecewise_polynomial",
    ],
)

cpplint()
