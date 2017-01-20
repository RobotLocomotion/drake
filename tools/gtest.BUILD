# -*- python -*-

cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"],
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h",
    ]),
    copts = ["-Wno-unused-const-variable"],
    defines = [
        "GTEST_DONT_DEFINE_FAIL=1",
        "GTEST_DONT_DEFINE_SUCCEED=1",
        "GTEST_DONT_DEFINE_TEST=1",
    ],
    includes = ["include"],
    linkopts = ["-pthread"],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
