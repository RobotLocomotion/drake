# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "drakeGeometryUtil.cpp",
        "drakeUtil.cpp",
    ],
    hdrs = [
        "drakeGeometryUtil.h",
        "drakeUtil.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
    ],
)

cpplint()
