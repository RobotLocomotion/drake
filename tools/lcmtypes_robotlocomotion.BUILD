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
    "lcm_c_library",
    "lcm_cc_library",
    "lcm_java_library",
    "lcm_py_library",
)

LCM_SRCS = glob(["lcmtypes/*.lcm"])

lcm_c_library(
    name = "lcmtypes_robotlocomotion_c",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@lcmtypes_bot2_core//:lcmtypes_bot2_core_c"],
)

lcm_cc_library(
    name = "lcmtypes_robotlocomotion",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@lcmtypes_bot2_core"],
)

lcm_java_library(
    name = "lcmtypes_robotlocomotion_java",
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@lcmtypes_bot2_core//:lcmtypes_bot2_core_java"],
)

lcm_py_library(
    name = "lcmtypes_robotlocomotion_py",
    imports = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = LCM_SRCS,
    deps = ["@lcmtypes_bot2_core//:lcmtypes_bot2_core_py"],
)

CMAKE_PACKAGE = "robotlocomotion-lcmtypes"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

install(
    name = "install",
    workspace = CMAKE_PACKAGE,
    targets = [
        ":lcmtypes_robotlocomotion",
        ":lcmtypes_robotlocomotion_c",
        ":lcmtypes_robotlocomotion_java",
        ":lcmtypes_robotlocomotion_py",
    ],
    py_strip_prefix = ["lcmtypes"],
    guess_hdrs = "PACKAGE",
    docs = ["LICENSE.txt"],
    rename = {
        "share/java/liblcmtypes_robotlocomotion_java.jar": "lcmtypes_robotlocomotion.jar",  # noqa
    },
    deps = [":install_cmake_config"],
)

# See https://github.com/RobotLocomotion/lcmtypes/issues/2 and
# https://github.com/openhumanoids/bot_core_lcmtypes/issues/33.
exports_files(
    ["LICENSE.txt"],
    visibility = ["@lcmtypes_bot2_core//:__pkg__"],
)
