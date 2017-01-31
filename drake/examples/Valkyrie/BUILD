# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "valkyrie_constants",
    srcs = ["valkyrie_constants.cc"],
    hdrs = ["valkyrie_constants.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
    ],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.dae",
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

cpplint()
