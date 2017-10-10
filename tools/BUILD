# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "exports_create_cps_scripts",
    "install_cmake_config",
)
load("//tools/lint:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

# Depend on this when you need to call clang-format.  (We don't have a label
# for the binary yet, but this will stand in for it in the meantime.)
filegroup(
    name = "clang-format",
    srcs = ["//:.clang-format"],
)

sh_binary(
    name = "drake_visualizer",
    srcs = select({
        "//tools/cc_toolchain:apple": [
            "drake_visualizer_apple.sh",
        ],
        "//conditions:default": [
            "drake_visualizer_linux.sh",
        ],
    }),
    data = [
        "//drake/bindings:pydrake",
        "//drake/examples:prod_models",
        "//drake/lcmtypes:lcmtypes_py",
        "@drake_visualizer",
        "@optitrack_driver//lcmtypes:py_optitrack_lcmtypes",
    ],
)

py_binary(
    name = "cmake_configure_file",
    srcs = ["cmake_configure_file.py"],
)

py_library(
    name = "cpsutils",
    srcs = ["cpsutils.py"],
    deps = ["@pycps//:cps"],
)

exports_files(
    ["install.py.in"],
    visibility = ["//visibility:public"],
)

exports_files(
    ["nlopt-gen-hpp.sh"],
    visibility = ["@nlopt//:__pkg__"],
)

exports_files(
    ["ipopt_build_with_autotools.sh"],
    visibility = ["@ipopt_robotlocomotion//:__pkg__"],
)

exports_files(
    [
        "install/libbot/bot-procman-sheriff",
        "install/libbot/build_prefix.py",
    ],
    visibility = ["@libbot//:__pkg__"],
)

exports_files(
    [
        "install/kuka_plan_runner/kuka_plan_runner.sh",
    ],
    visibility = ["//drake/examples/kuka_iiwa_arm:__pkg__"],
)

exports_create_cps_scripts([
    "bullet",
    "ccd",
    "eigen",
    "fcl",
    "fmt",
    "ignition_math",
    "ignition_rndf",
    "lcm",
    "nlopt",
    "octomap",
    "pybind11",
    "sdformat",
    "spdlog",
    "tinyobjloader",
    "yaml_cpp",
])

exports_files(
    ["bot2-core-lcmtypes.cps"],
    visibility = ["@lcmtypes_bot2_core//:__pkg__"],
)

exports_files(
    ["libbot.cps"],
    visibility = ["@libbot//:__pkg__"],
)

exports_files(
    ["robotlocomotion-lcmtypes.cps"],
    visibility = ["@lcmtypes_robotlocomotion//:__pkg__"],
)

cmake_config(
    cps_file_name = "drake.cps",
    package = "drake",
)

install_cmake_config(
    name = "install",
    package = "drake",
    versioned = 0,
    visibility = ["//visibility:public"],
)

# === config_setting rules ===

config_setting(
    name = "with_gurobi",
    values = {"define": "WITH_GUROBI=ON"},
)

config_setting(
    name = "with_mosek",
    values = {"define": "WITH_MOSEK=ON"},
)

config_setting(
    name = "with_snopt",
    values = {"define": "WITH_SNOPT=ON"},
)

# IPOPT is an open-source solver, and is included in the Drake build by
# default. The IPOPT solver is irrelvant to some users of MathematicalProgram,
# so we provide a hidden switch to shut it off for developers who don't
# actually need it.  This is not a supported configuration. Use at your own
# risk: --define=NO_IPOPT=ON
config_setting(
    name = "no_ipopt",
    values = {"define": "NO_IPOPT=ON"},
)

# NLOPT is an open-source solver, and is included in the Drake build by
# default. The NLOPT solver is irrelvant to some users of MathematicalProgram,
# so we provide a hidden switch to shut it off for developers who don't
# actually need it.  This is not a supported configuration. Use at your own
# risk: --define=NO_NLOPT=ON
config_setting(
    name = "no_nlopt",
    values = {"define": "NO_NLOPT=ON"},
)

add_lint_tests()
