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
    name = "spdlog",
    hdrs = glob(["include/spdlog/**"]),
    defines = [
        "HAVE_SPDLOG",
        "SPDLOG_FMT_EXTERNAL",
    ],
    includes = ["include"],
    linkopts = select({
        "@drake//tools:linux": ["-pthread"],
        # This is a bazel-default rule, and does not need @drake//
        "@//conditions:default": [],
    }),
    deps = ["@fmt"],
)

cmake_config(
    package = "spdlog",
    script = "@drake//tools:spdlog-create-cps.py",
    version_file = "CMakeLists.txt",
    deps = ["@fmt//:cps"],
)

install_cmake_config(package = "spdlog")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/spdlog",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/spdlog",
    hdr_strip_prefix = ["include/spdlog"],
    license_docs = ["LICENSE"],
    targets = [":spdlog"],
    deps = [":install_cmake_config"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE"],
    mode = "0644",
    package_dir = "spdlog",
)
