# -*- python -*-

cc_library(
    name = "ignition_math",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/*_TEST.cc"],
    ),
    hdrs = glob([
        "include/**/**/*.hh",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
