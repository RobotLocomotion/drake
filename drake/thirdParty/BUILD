# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spruce",
    srcs = ["bsd/spruce/spruce.cc"],
    hdrs = ["bsd/spruce/spruce.hh"],
    includes = ["bsd/spruce"],
    linkstatic = 1,
)

cc_library(
    name = "tinydir",
    hdrs = ["bsd/tinydir/tinydir.h"],
    linkstatic = 1,
)

cc_library(
    name = "tinyxml2",
    srcs = ["zlib/tinyxml2/tinyxml2.cpp"],
    hdrs = ["zlib/tinyxml2/tinyxml2.h"],
    linkstatic = 1,
)
