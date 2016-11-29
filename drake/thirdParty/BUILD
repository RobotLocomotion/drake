# -*- python -*

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spruce",
    srcs = ["bsd/spruce/src/spruce.cc"],
    hdrs = ["bsd/spruce/include/spruce.hh"],
    includes = ["bsd/spruce/include"],
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
