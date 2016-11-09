# -*- python -*-

cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    includes = ["include"],
    linkopts = ["-pthread"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
