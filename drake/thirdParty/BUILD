# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

load("//tools:drake.bzl", "drake_cc_library")

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
