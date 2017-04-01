# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(default_visibility = ["//visibility:public"])

# === test/ ===

drake_cc_googletest(
    name = "ignition_math_test",
    srcs = ["test/ignition_math_test.cc"],
    deps = [
        "@ignition_math",
    ],
)

cpplint()
