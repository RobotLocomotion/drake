# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "spdlog",
    hdrs = glob(["include/spdlog/**"]),
    defines = ["HAVE_SPDLOG"],
    includes = ["include"],
    linkopts = select({
        "@//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE"],
    package_dir = "spdlog",
)
