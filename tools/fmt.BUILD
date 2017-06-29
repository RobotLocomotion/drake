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
    docs = ["LICENSE.rst"],
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/fmt",
    hdr_strip_prefix = ["fmt"],
    targets = [":fmt"],
    deps = [":install_cmake_config"],
)
