# -*- python -*-

load(
    "@drake//tools/install:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stx",
    hdrs = ["stx/optional.hpp"],
    includes = ["."],
)

CMAKE_PACKAGE = "stx"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

install(
    name = "install",
    targets = [":stx"],
    hdr_dest = "include/stx",
    guess_hdrs = "PACKAGE",
    docs = [
        "LICENSE",
    ],
    deps = [
        ":install_cmake_config",
    ],
)
