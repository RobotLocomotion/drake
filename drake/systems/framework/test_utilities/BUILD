# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "test_utilities",
    testonly = 1,
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":pack_value",
    ],
)

drake_cc_library(
    name = "pack_value",
    testonly = 1,
    srcs = [],
    hdrs = ["pack_value.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework:value",
    ],
)

cpplint()
