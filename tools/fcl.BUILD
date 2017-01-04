# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Lets other packages inspect the CMake code, e.g., for the version number.
filegroup(
    name = "cmakelists_with_version",
    srcs = ["CMakeModules/FCLVersion.cmake"],
    visibility = ["//visibility:public"],
)

# Generates config.h based on the version numbers in CMake code.
cmake_configure_file(
    name = "config",
    src = "include/fcl/config.h.in",
    out = "include/fcl/config.h",
    cmakelists = [
        ":cmakelists_with_version",
        "@octomap//:cmakelists_with_version",
    ],
    defines = ["FCL_HAVE_OCTOMAP"],
    visibility = [],
)

# Generates the entire FCL library except the fcl/fcl.h generated header.
# The globbed srcs= and hdrs= matches upstream's explicit globs of the same.
cc_library(
    name = "lib_without_fclh",
    srcs = glob(["src/**/*.cpp"]),
    hdrs = glob(["include/**/*.h"]) + [
        "include/fcl/config.h",  # From :config above.
    ],
    includes = ["include"],
    linkstatic = 1,
    visibility = [],
    deps = [
        "@ccd//:lib",
        "@eigen//:eigen",
        "@octomap//:lib",
    ],
)

# Generates fcl.h, which consists of #include statements for *all* of the other
# headers in the library (!!!).  The first line is '#pragma once' followed by
# one line like '#include "fcl/common/types.h"' for each non-generated header.
genrule(
    name = "fclh_genrule",
    srcs = glob(["include/**/*.h"]),
    outs = ["include/fcl/fcl.h"],
    cmd = "(" + (
        "echo '#pragma once' && " +
        "echo '$(SRCS)' | tr ' ' '\\n' | " +
        "sed 's|.*include/\(.*\)|#include \\\"\\1\\\"|g'"
    ) + ") > '$@'",
    visibility = [],
)

# Generates the library exported to users.
cc_library(
    name = "lib",
    hdrs = ["include/fcl/fcl.h"],  # From :fclh_genrule above.
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [":lib_without_fclh"],
)
