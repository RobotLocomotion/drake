# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "dragway",
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
        "//drake/math:saturate",
    ],
)

drake_cc_binary(
    name = "dragway_to_urdf",
    srcs = ["dragway_to_urdf.cc"],
    deps = [
        ":dragway",
        "//drake/automotive/maliput/utility",
        "//drake/common",
        "//drake/common:text_logging_gflags",
        "//drake/thirdParty:spruce",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "dragway_test",
    size = "small",
    srcs = ["test/dragway_test.cc"],
    deps = [
        ":dragway",
    ],
)

cpplint()
