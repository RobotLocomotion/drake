# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "crossroad",
    srcs = [
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "road_geometry.cc",
        "segment.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "road_geometry.h",
        "segment.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
    ],
)

drake_cc_binary(
    name = "crossroad_to_urdf",
    srcs = ["crossroad_to_urdf.cc"],
    deps = [
        ":crossroad",
        "//drake/automotive/maliput/utility",
        "//drake/common",
        "//drake/common:text_logging_gflags",
        "//drake/thirdParty:spruce",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "crossroad_test",
    size = "small",
    srcs = ["test/crossroad_test.cc"],
    deps = [
        ":crossroad",
    ],
)

cpplint()
