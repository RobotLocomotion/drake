package(default_visibility = ["//visibility:public"])

load("@//tools:lcm.bzl", "lcm_cc_library", "lcm_py_library")

lcm_cc_library(
    name = "lib",
    includes = ["lcmtypes"],
    lcm_package = "robotlocomotion",
    lcm_srcs = glob(["lcmtypes/*.lcm"]),
    linkstatic = 1,
)
