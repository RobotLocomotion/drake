# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

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
    doc_dest = "share/doc/tinyobjloader",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/tinyobjloader",
    license_docs = ["LICENSE"],
    targets = [":tinyobjloader"],
    deps = [":install_cmake_config"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE"],
    mode = "0644",
    package_dir = "tinyobjloader",
)
