# -*- python -*-

package(default_visibility = ["//visibility:public"])

load("@drake//tools:lcm.bzl", "lcm_cc_library", "lcm_py_library")

lcm_cc_library(
    name = "bot_core_lcmtypes",
    includes = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = glob(["lcmtypes/*.lcm"]),
    # Input file "lcmtypes/bot_core_foo_t.lcm" yields struct "foo_t".
    lcm_structs = [f[18:-4] for f in glob(["lcmtypes/*.lcm"])],
    linkstatic = 1,
)

lcm_py_library(
    name = "py",
    imports = ["lcmtypes"],
    lcm_package = "bot_core",
    lcm_srcs = glob(["lcmtypes/*.lcm"]),
    # Input file "lcmtypes/bot_core_foo_t.lcm" yields struct "foo_t".
    lcm_structs = [f[18:-4] for f in glob(["lcmtypes/*.lcm"])],
)
