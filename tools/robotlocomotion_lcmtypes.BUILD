# -*- python -*-

package(default_visibility = ["//visibility:public"])

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)
load(
    "@drake//tools:lcm.bzl",
    "lcm_c_aggregate_header",
    "lcm_c_library",
    "lcm_cc_library",
    "lcm_java_library",
    "lcm_py_library",
)

LCM_SRCS = glob(["lcmtypes/*.lcm"])

lcm_c_aggregate_header(
    name = "robotlocomotion_lcmtypes_c_aggregate_header",
    out = "lcmtypes/robotlocomotion.h",
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    visibility = ["//visibility:private"],
)

lcm_c_library(
    name = "robotlocomotion_lcmtypes_c",
    aggregate_hdr = ":robotlocomotion_lcmtypes_c_aggregate_header",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@bot_core_lcmtypes//:bot_core_lcmtypes_c"],
)

lcm_cc_library(
    name = "robotlocomotion_lcmtypes",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@bot_core_lcmtypes"],
)

lcm_java_library(
    name = "robotlocomotion_lcmtypes_java",
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@bot_core_lcmtypes//:bot_core_lcmtypes_java"],
)

lcm_py_library(
    name = "robotlocomotion_lcmtypes_py",
    imports = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@bot_core_lcmtypes//:bot_core_lcmtypes_py"],
)

CMAKE_PACKAGE = "robotlocomotion-lcmtypes"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

install(
    name = "install",
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    guess_hdrs = "PACKAGE",
    license_docs = ["LICENSE.txt"],
    py_strip_prefix = ["lcmtypes"],
    targets = [
        ":robotlocomotion_lcmtypes",
        ":robotlocomotion_lcmtypes_c",
        ":robotlocomotion_lcmtypes_java",
        ":robotlocomotion_lcmtypes_py",
    ],
    deps = [":install_cmake_config"],
)

# See https://github.com/RobotLocomotion/lcmtypes/issues/2 and
# https://github.com/openhumanoids/bot_core_lcmtypes/issues/33.
exports_files(
    ["LICENSE.txt"],
    visibility = ["@bot_core_lcmtypes//:__pkg__"],
)
