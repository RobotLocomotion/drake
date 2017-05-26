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
        "@drake//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
    deps = ["@fmt"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE"],
    mode = "0644",
    package_dir = "spdlog",
)
