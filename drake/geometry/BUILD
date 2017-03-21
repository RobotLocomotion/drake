# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

cpplint()
