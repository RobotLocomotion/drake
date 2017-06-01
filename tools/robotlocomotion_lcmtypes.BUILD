# -*- python -*-

package(default_visibility = ["//visibility:public"])

load("@drake//tools:install.bzl", "cmake_config", "install", "install_cmake_config")
load("@drake//tools:lcm.bzl", "lcm_cc_library", "lcm_py_library")

lcm_cc_library(
    name = "robotlocomotion_lcmtypes",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = glob(["lcmtypes/*.lcm"]),
)

lcm_py_library(
    name = "robotlocomotion_lcmtypes_py",
    imports = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = glob(["lcmtypes/*.lcm"]),
)

CMAKE_PACKAGE = "robotlocomotion-lcmtypes"

cmake_config(package = CMAKE_PACKAGE)
install_cmake_config(package = CMAKE_PACKAGE)

install(
    name = "install",
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    guess_hdrs = "PACKAGE",
    hdr_dest = "include/" + CMAKE_PACKAGE,
    license_docs = ["LICENSE.txt"],
    py_strip_prefix = ["lcmtypes"],
    targets = [
        ":robotlocomotion_lcmtypes_py",
        ":robotlocomotion_lcmtypes",
    ],
    deps = [":install_cmake_config"],
)

# See https://github.com/RobotLocomotion/lcmtypes/issues/2 and
# https://github.com/openhumanoids/bot_core_lcmtypes/issues/33.
exports_files(
    ["LICENSE.txt"],
    visibility = ["@bot_core_lcmtypes//:__pkg__"],
)
