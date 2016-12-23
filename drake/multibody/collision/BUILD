# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

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

cc_library(
    name = "collision_api",
    hdrs = ["drake_collision.h"],
    deps = [
        ":model",
    ],
)

cc_library(
    name = "bullet_collision",
    srcs = [
        "bullet_model.cc",
        "bullet_model.h",
        "drake_collision.cc",
    ],
    defines = ["BULLET_COLLISION"],
    linkstatic = 1,
    deps = [
        ":collision_api",
        "@bullet//:lib",
    ],
)

# TODO(jwnimmer-tri) Remove this once CMake is no longer using it (and thus we
# can delete unusable_model.*).  This cc_library only exists a sanity check to
# ensure the file still compiles.
cc_library(
    name = "unusable_collision",
    srcs = [
        "drake_collision.cc",
        "unusable_model.cc",
        "unusable_model.h",
    ],
    linkstatic = 1,
    visibility = ["//visibility:private"],
    deps = [
        ":collision_api",
    ],
)

# By default, supply a bullet-only model.  As we gain more collision library
# choices, we may want to be more subtle here.
alias(
    name = "collision",
    actual = ":bullet_collision",
)

cc_googletest(
    name = "model_test",
    data = [":test_models"],
    deps = [
        ":collision",
        "//drake/common:drake_path",
    ],
)

filegroup(
    name = "test_models",
    testonly = 1,
    srcs = glob([
        "test/**/*.obj",
        "test/**/*.sdf",
        "test/**/*.urdf",
        "test/**/*.xml",
    ]),
)
