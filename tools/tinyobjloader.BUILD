# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

cc_library(
    name = "tinyobjloader",
    srcs = [
        "tiny_obj_loader.cc",
    ],
    hdrs = [
        "tiny_obj_loader.h",
    ],
    includes = ["."],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE"],
    mode = "0644",
    package_dir = "tinyobjloader",
    visibility = ["//visibility:public"],
)
