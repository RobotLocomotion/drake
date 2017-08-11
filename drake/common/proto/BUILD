# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")
load("@protobuf//:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "matlab_rpc",
    testonly = 1,
    srcs = [
        "matlab_rpc.proto",
    ],
)

drake_cc_library(
    name = "call_matlab",
    testonly = 1,
    srcs = ["call_matlab.cc"],
    hdrs = ["call_matlab.h"],
    deps = [
        ":matlab_rpc",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "protobuf",
    srcs = ["protobuf.cc"],
    hdrs = ["protobuf.h"],
    deps = [
        "@protobuf",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "call_matlab_test",
    deps = [
        ":call_matlab",
    ],
)

drake_cc_googletest(
    name = "protobuf_test",
    data = [
        "test/test_string.txt",
    ],
    deps = [
        ":protobuf",
    ],
)

add_lint_tests()
