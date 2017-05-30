# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load("@drake//tools:install.bzl", "cmake_config", "install", "install_cmake_config")
load("@//tools:cmake_configure_file.bzl", "cmake_configure_file")

package(
    default_visibility = ["//visibility:public"],
)

# Generates config.h based on the defines= we want in Drake.
# Keep defines in sync with Components.ccd.Definitions in
# ccd-create-cps.py.
cmake_configure_file(
    name = "config",
    src = "src/ccd/config.h.cmake.in",
    out = "src/ccd/config.h",
    defines = ["CCD_DOUBLE"],
    visibility = [],
)

# Lets other packages inspect the CMake code, e.g., for the version number.
# Guessing headers with
#       install(...
#          guess_hdrs = "PACKAGE",
#         )
# fails and lists internal headers in addition to the public headers.

filegroup(
    name = "ccd_hdrs",
    srcs = glob([
        "src/ccd/*.h",
    ]) + [
        "src/ccd/config.h",  # From :config above.
    ],
    visibility = ["//visibility:public"],
)

# Generates the library exported to users.  Upstream's CMake code lists out all
# sources instead of globbing, but conveniently puts the public headers in one
# place and private sources and headers in another; we'll use globbing here.
cc_library(
    name = "ccd",
    srcs = glob([
        "src/*.c",
        "src/*.h",
    ]),
    hdrs = [":ccd_hdrs"],
    copts = ["-Wno-all"],
    includes = ["src"],
    visibility = ["//visibility:public"],
)

cmake_config(
    package = "ccd",
    script = "@drake//tools:ccd-create-cps.py",
    version_file = "CMakeLists.txt",
)

install_cmake_config(package = "ccd")  # Creates rule :install_cmake_config.

install(
    name = "install",
    hdrs = [":ccd_hdrs"],
    doc_dest = "share/doc/ccd",
    hdr_dest = "include/ccd",
    hdr_strip_prefix = ["**/"],
    license_docs = ["BSD-LICENSE"],
    targets = [":ccd"],
    deps = [":install_cmake_config"],
)
