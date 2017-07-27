# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")
load("//tools:install_data.bzl", "install_data")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "atlas_util",
    srcs = ["atlasUtil.cpp"],
    hdrs = ["atlasUtil.h"],
    deps = [
        "//drake/common",
    ],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

install_data("share/drake/drake/examples/atlas")

cpplint()
