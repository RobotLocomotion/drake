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
    name = "type_safe_int_id",
    srcs = [],
    hdrs = ["type_safe_int_id.h"],
    deps = ["//drake/common"],
)

drake_cc_googletest(
    name = "type_safe_int_id_test",
    deps = [":type_safe_int_id"],
)

cpplint()
