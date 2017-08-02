# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_library")
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "atlas_util",
    srcs = ["atlasUtil.cpp"],
    hdrs = ["atlasUtil.h"],
    deps = [
        "//drake/common",
    ],
)

install_data()

add_lint_tests()
