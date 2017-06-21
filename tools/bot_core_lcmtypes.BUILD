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

# Input file "lcmtypes/bot_core_foo_t.lcm" yields struct "foo_t".
LCM_STRUCTS = [
    f.replace("lcmtypes/bot_core_", "").replace(".lcm", "")
    for f in LCM_SRCS
]

lcm_c_aggregate_header(
    name = "bot_core_lcmtypes_c_aggregate_header",
    out = "lcmtypes/bot_core.h",
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
    visibility = ["//visibility:private"],
)

lcm_c_library(
    name = "bot_core_lcmtypes_c",
    aggregate_hdr = ":bot_core_lcmtypes_c_aggregate_header",
    includes = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_cc_library(
    name = "bot_core_lcmtypes",
    includes = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_java_library(
    name = "bot_core_lcmtypes_java",
    lcm_package = "bot_core",
    lcm_srcs = LCM_SRCS,
    lcm_structs = LCM_STRUCTS,
)

lcm_py_library(
    name = "bot_core_lcmtypes_py",
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

# For license_docs, see  https://github.com/RobotLocomotion/lcmtypes/issues/2
# and https://github.com/openhumanoids/bot_core_lcmtypes/issues/33.
install(
    name = "install",
    doc_dest = "share/doc/" + CMAKE_PACKAGE,
    guess_hdrs = "PACKAGE",
    license_docs = ["@lcmtypes_robotlocomotion//:LICENSE.txt"],
    py_strip_prefix = ["lcmtypes"],
    targets = [
        ":bot_core_lcmtypes",
        ":bot_core_lcmtypes_c",
        ":bot_core_lcmtypes_java",
        ":bot_core_lcmtypes_py",
    ],
    deps = [":install_cmake_config"],
)
