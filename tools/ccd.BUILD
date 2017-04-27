# -*- python -*-

load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

# Generates config.h based on the defines= we want in Drake.
cmake_configure_file(
    name = "config",
    src = "src/ccd/config.h.cmake.in",
    out = "src/ccd/config.h",
    defines = ["CCD_DOUBLE"],
    visibility = [],
)

# Generates the library exported to users.  Upstream's CMake code lists out all
# sources instead of globbing, but conveniently puts the public headers in one
# place and private sources and headers in another; we'll use globbing here.
cc_library(
    name = "lib",
    srcs = glob([
        "src/*.c",
        "src/*.h",
    ]),
    hdrs = glob([
        "src/ccd/*.h",
    ]) + [
        "src/ccd/config.h",  # From :config above.
    ],
    copts = ["-Wno-all"],
    includes = ["src"],
    visibility = ["//visibility:public"],
)
