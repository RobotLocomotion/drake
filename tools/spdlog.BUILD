# -*- python -*-

cc_library(
    name = "spdlog",
    hdrs = glob(["include/spdlog/**"]),
    defines = ["HAVE_SPDLOG"],
    includes = ["include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)
