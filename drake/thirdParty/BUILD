# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

load("//tools:drake.bzl", "drake_cc_library")
load("//tools:bazel_lint.bzl", "bazel_lint")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "spruce",
    srcs = ["bsd/spruce/spruce.cc"],
    hdrs = ["bsd/spruce/spruce.hh"],
    includes = ["bsd/spruce"],
)

drake_cc_library(
    name = "tinydir",
    hdrs = ["bsd/tinydir/tinydir.h"],
)

drake_cc_library(
    name = "tinyxml2",
    srcs = ["zlib/tinyxml2/tinyxml2.cpp"],
    hdrs = ["zlib/tinyxml2/tinyxml2.h"],
)

drake_cc_library(
    name = "stx",
    hdrs = ["boost/tcbrindle_cpp17_headers/stx/optional.hpp"],
)

# Most BUILD files should say add_lint_tests() here, but insetad we only check
# Bazel lint -- we don't want to lint the style choices in third-party code!
bazel_lint()
