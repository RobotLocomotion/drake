# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shapes",
    srcs = [
        "element.cc",
        "geometry.cc",
        "visual_element.cc",
    ],
    hdrs = [
        "drake_shapes.h",
        "element.h",
        "geometry.h",
        "visual_element.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/thirdParty:spruce",
    ],
)
