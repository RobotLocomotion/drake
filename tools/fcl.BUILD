# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load(
    "@drake//tools:generate_include_header.bzl",
    "drake_generate_include_header",
)
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(
    default_visibility = ["//visibility:public"],
)

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
    visibility = ["//visibility:private"],
)

# Generates fcl.h, which consists of #include statements for *all* of the other
# headers in the library (!!!).  The first line is '#pragma once' followed by
# one line like '#include "fcl/common/types.h"' for each non-generated header.
drake_generate_include_header(
    name = "fcl_h_genrule",
    out = "include/fcl/fcl.h",
    hdrs = glob(["include/**/*.h"]),
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
    deps = [
        "@ccd",
        "@eigen",
        "@octomap",
    ],
)

cmake_config(
    package = "fcl",
    script = "@drake//tools:fcl-create-cps.py",
    version_file = "CMakeModules/FCLVersion.cmake",
    deps = [
        "@ccd//:cps",
        "@eigen//:cps",
        "@octomap//:cps",
    ],
)

install_cmake_config(package = "fcl")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/fcl",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/fcl",
    hdr_strip_prefix = ["include/fcl"],
    license_docs = glob(["LICENSE"]),
    targets = [":fcl"],
    deps = [":install_cmake_config"],
)
