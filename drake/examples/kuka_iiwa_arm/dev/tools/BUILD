# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

drake_cc_library(
    name = "moving_average_filter",
    srcs = ["moving_average_filter.cc"],
    hdrs = ["moving_average_filter.h"],
    deps = ["//drake/common"],
)

# === test/ ===

drake_cc_googletest(
    name = "moving_average_filter_test",
    srcs = ["test/moving_average_filter_test.cc"],
    deps = [
        ":moving_average_filter",
        "//drake/common",
    ],
)

cpplint()
