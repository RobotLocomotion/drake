# -*- python -*-

cc_library(
    name = "spdlog",
    hdrs = glob(["include/spdlog/**"]),
    defines = ["HAVE_SPDLOG"],
    includes = ["include"],
    linkopts = select({
        "@//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
)
