# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "tinyobjloader",
    srcs = [
        "tiny_obj_loader.cc",
    ],
    hdrs = [
        "tiny_obj_loader.h",
    ],
    includes = ["."],
    linkstatic = 0,
)

cmake_config(
    package = "tinyobjloader",
    script = "@drake//tools:tinyobjloader-create-cps.py",
    version_file = "CMakeLists.txt",
)

# Creates rule :install_cmake_config.
install_cmake_config(package = "tinyobjloader")

install(
    name = "install",
    targets = [":tinyobjloader"],
    hdr_dest = "include/tinyobjloader",
    guess_hdrs = "PACKAGE",
    docs = ["LICENSE"],
    deps = [":install_cmake_config"],
)
