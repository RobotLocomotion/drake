# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Generates config.h based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "include/fcl/config.h.in",
    out = "include/fcl/config.h",
    cmakelists = [
        "CMakeModules/FCLVersion.cmake",
        "@octomap//:cmakelists_with_version",
    ],
    defines = ["FCL_HAVE_OCTOMAP"],
)

# Generates fcl.h, which consists of #include statements for *all* of the other
# headers in the library (!!!).  The first line is '#pragma once' followed by
# one line like '#include "fcl/common/types.h"' for each non-generated header.
genrule(
    name = "fcl_h_genrule",
    srcs = glob(["include/**/*.h"]),
    outs = ["include/fcl/fcl.h"],
    cmd = "(" + (
        "echo '#pragma once' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\\"\\1\\\"|g'"
    ) + ") > '$@'",
)

# The globbed srcs= and hdrs= matches upstream's explicit globs of the same.
cc_library(
    name = "fcl",
    srcs = glob(["src/**/*.cpp"]),
    hdrs = glob(["include/**/*.h"]) + [
        ":config",
        ":fcl_h_genrule",
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ccd",
        "@eigen",
        "@octomap",
    ],
)
