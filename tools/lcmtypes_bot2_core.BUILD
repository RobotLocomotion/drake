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

# Input file "lcmtypes/bot_core_foo_t.lcm" yields struct "foo_t".
LCM_STRUCTS = [
    f.replace("lcmtypes/bot_core_", "").replace(".lcm", "")
    for f in LCM_SRCS
]

lcm_c_library(
    name = "lcmtypes_bot2_core_c",
    includes = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_cc_library(
    name = "lcmtypes_bot2_core",
    aggregate_hdr = "lcmtypes/bot_core.hpp",
    includes = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_java_library(
    name = "lcmtypes_bot2_core_java",
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_py_library(
    name = "lcmtypes_bot2_core_py",
    imports = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

CMAKE_PACKAGE = "bot2-core-lcmtypes"

cmake_config(package = CMAKE_PACKAGE)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

# For docs, see  https://github.com/RobotLocomotion/lcmtypes/issues/2
# and https://github.com/openhumanoids/bot_core_lcmtypes/issues/33.
install(
    name = "install",
    workspace = CMAKE_PACKAGE,
    targets = [
        ":lcmtypes_bot2_core",
        ":lcmtypes_bot2_core_c",
        ":lcmtypes_bot2_core_java",
        ":lcmtypes_bot2_core_py",
    ],
    py_strip_prefix = ["lcmtypes"],
    guess_hdrs = "PACKAGE",
    docs = ["@lcmtypes_robotlocomotion//:LICENSE.txt"],
    rename = {
        "share/java/liblcmtypes_bot2_core_java.jar": "lcmtypes_bot2_core.jar",
    },
    deps = [":install_cmake_config"],
)
