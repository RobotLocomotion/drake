# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library", "drake_cc_binary")

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
    linkstatic = 1,
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
    ],
)

# === test/ ===

cc_test(
    name = "dragway_test",
    size = "small",
    srcs = ["test/dragway_test.cc"],
    deps = [
        ":dragway",
        "@gtest//:main",
    ],
)

cpplint()
