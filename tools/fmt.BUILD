# -*- python -*-

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")
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
    name = "fmt",
    srcs = glob(["fmt/*.cc"]),
    hdrs = glob(["fmt/*.h"]),
    includes = ["."],
)

cmake_config(
    package = "fmt",
    script = "@drake//tools:fmt-create-cps.py",
    version_file = "CMakeLists.txt",
)

install_cmake_config(package = "fmt")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/fmt",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/fmt",
    hdr_strip_prefix = ["fmt"],
    license_docs = ["LICENSE.rst"],
    targets = [":fmt"],
    deps = [":install_cmake_config"],
)

pkg_tar(
    name = "license",
    extension = "tar.gz",
    files = ["LICENSE.rst"],
    mode = "0644",
    package_dir = "fmt",
)
