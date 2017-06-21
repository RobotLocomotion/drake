# -*- python -*-

load("@drake//tools:cmake_configure_file.bzl", "cmake_configure_file")
load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(
    default_visibility = ["//visibility:public"],
)

# Generates config.h based on the defines= we want in Drake.
cmake_configure_file(
    name = "config",
    src = "src/ccd/config.h.cmake.in",
    out = "src/ccd/config.h",
    defines = ["CCD_DOUBLE"],
    visibility = ["//visibility:private"],
)

# Guessing headers with
#       install(...
#          guess_hdrs = "PACKAGE",
#         )
# fails and lists internal headers in addition to the public headers.
CCD_PUBLIC_HEADERS = [
    "src/ccd/ccd.h",
    "src/ccd/config.h",
    "src/ccd/compiler.h",
    "src/ccd/quat.h",
    "src/ccd/vec3.h",
]

# Generates the library exported to users.  Upstream's CMake code lists out all
# sources instead of globbing, but conveniently puts the public headers in one
# place and private sources and headers in another; we'll use globbing here.
cc_library(
    name = "ccd",
    srcs = glob([
        "src/*.c",
        "src/*.h",
    ]),
    hdrs = CCD_PUBLIC_HEADERS,
    copts = ["-Wno-all"],
    includes = ["src"],
)

cmake_config(
    package = "ccd",
    script = "@drake//tools:ccd-create-cps.py",
    version_file = "CMakeLists.txt",
)

install_cmake_config(package = "ccd")  # Creates rule :install_cmake_config.

install(
    name = "install",
    hdrs = CCD_PUBLIC_HEADERS,
    doc_dest = "share/doc/ccd",
    hdr_dest = "include/ccd",
    hdr_strip_prefix = ["**/"],
    license_docs = ["BSD-LICENSE"],
    targets = [":ccd"],
    deps = [":install_cmake_config"],
)
