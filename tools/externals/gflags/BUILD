# -*- python -*-

load("//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

alias(
    name = "gflags",
    actual = "@com_github_gflags_gflags//:gflags",
)

alias(
    name = "COPYING.txt",
    actual = "@com_github_gflags_gflags//:COPYING.txt",
)

cmake_config(
    package = "gflags",
)

install_cmake_config(package = "gflags")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/gflags",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/gflags",
    hdr_strip_prefix = ["include"],
    license_docs = [":COPYING.txt"],
    targets = [":gflags"],
    deps = [":install_cmake_config"],
)
