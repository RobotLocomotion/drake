# -*- python -*-

load("@drake//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "yaml_cpp",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.h",
    ]),
    hdrs = glob([
        "include/**/*.h",
    ]),
    includes = ["include"],
)

cmake_config(
    package = "yaml-cpp",
    script = "@drake//tools:yaml_cpp-create-cps.py",
    version_file = "CMakeLists.txt",
)

install_cmake_config(package = "yaml-cpp")  # Creates rule :install_cmake_config.

install(
    name = "install",
    doc_dest = "share/doc/yaml-cpp",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include",
    hdr_strip_prefix = ["include"],
    license_docs = ["LICENSE"],
    targets = [":yaml_cpp"],
    deps = [":install_cmake_config"],
)
