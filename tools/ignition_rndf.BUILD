# -*- python -*-

cc_library(
    name = "ignition_rndf",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/*_TEST.cc"],
    ),
    hdrs = glob([
        "include/**/*.hh",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@ignition_math//:lib",
    ],
)
