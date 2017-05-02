# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "spdlog",
    hdrs = glob(["include/spdlog/**"]),
    defines = [
        "HAVE_SPDLOG",
        "SPDLOG_FMT_EXTERNAL",
    ],
    includes = ["include"],
    linkopts = select({
        "@//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
    deps = ["@fmt//:fmt"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    mode = "0644",
    files = ["LICENSE"],
    package_dir = "spdlog",
)
