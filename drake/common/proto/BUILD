# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("@protobuf//:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "matlab_rpc",
    srcs = [
        "matlab_rpc.proto",
    ],
    testonly = 1,
)

drake_cc_library(
    name = "call_matlab",
    testonly = 1,
    srcs = ["call_matlab.cc"],
    hdrs = ["call_matlab.h"],
    deps = [
        "//drake/common",
        ":matlab_rpc",
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

cpplint()
