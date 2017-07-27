# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:install_data.bzl", "install_data")

drake_cc_binary(
    name = "zmp_example",
    testonly = 1,
    srcs = ["zmp_example.cc"],
    deps = [
        "//drake/systems/controllers:zmp_test_util",
        "//drake/common/proto:call_matlab",
    ],
)

install_data("share/drake/drake/examples/zmp")

cpplint()
