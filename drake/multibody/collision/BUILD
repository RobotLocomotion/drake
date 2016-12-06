# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model",
    srcs = [
        "element.cc",
        "model.cc",
    ],
    hdrs = [
        "element.h",
        "model.h",
        "point_pair.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:sorted_vectors_have_intersection",
        "//drake/multibody/shapes",
    ],
)

# TODO(jwnimmer-tri) Need to add bullet support.
# Right now this library is only "unusuable collision".
cc_library(
    name = "collision",
    srcs = [
        "drake_collision.cc",
        "unusable_model.cc",
        "unusable_model.h",
    ],
    hdrs = ["drake_collision.h"],
    linkstatic = 1,
    deps = [":model"],
)

filegroup(
    name = "test_models",
    srcs = glob([
        "test/**/*.obj",
        "test/**/*.sdf",
        "test/**/*.urdf",
        "test/**/*.xml",
    ]),
)
