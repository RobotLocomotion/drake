# -*- python -*-

load("//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

cmake_config(
    cps_file_name = "gflags.cps",
    package = "gflags",
)

install_cmake_config(package = "gflags")  # Creates rule :install_cmake_config.

install(
    name = "install",
    allowed_externals = ["@com_github_gflags_gflags//:gflags"],
    doc_dest = "share/doc/gflags",
    guess_hdrs = "PACKAGE",
    guess_hdrs_exclude = [
        "config.h",
        "src/mutex.h",
        "src/util.h",
    ],
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    license_docs = ["@com_github_gflags_gflags//:COPYING.txt"],
    targets = ["@com_github_gflags_gflags//:gflags"],
    deps = [":install_cmake_config"],
)
