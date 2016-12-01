# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "api",
    srcs = [
        "road_geometry.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "lane_data.h",
        "road_geometry.h",
        "segment.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)
