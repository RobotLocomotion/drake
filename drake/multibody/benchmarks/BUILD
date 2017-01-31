# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "acrobot",
    srcs = ["acrobot/acrobot.cc"],
    hdrs = ["acrobot/acrobot.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:extract_double",
        "//drake/multibody:rigid_body_tree",
    ],
)

filegroup(
    name = "acrobot_models",
    testonly = 1,
    srcs = glob([
        "acrobot/*.obj",
        "acrobot/*.sdf",
        "acrobot/*.urdf",
        "acrobot/*.xml",
    ]),
)

cpplint()
