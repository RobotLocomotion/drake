# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "acrobot",
    srcs = ["acrobot.cc"],
    hdrs = ["acrobot.h"],
    deps = [
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

filegroup(
    name = "models",
    testonly = 1,
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

cpplint()
