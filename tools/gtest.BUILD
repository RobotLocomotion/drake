# -*- python -*-

cc_library(
    name = "main",
    srcs = glob(
        ["googletest/src/*.cc"],
        exclude = ["googletest/src/gtest-all.cc"],
    ),
    hdrs = glob([
        "googletest/include/**/*.h",
        "googletest/src/*.h",
    ]),
    copts = ["-Wno-unused-const-variable"],
    defines = [
        "GTEST_DONT_DEFINE_FAIL=1",
        "GTEST_DONT_DEFINE_SUCCEED=1",
        "GTEST_DONT_DEFINE_TEST=1",
    ],
    includes = [
        "googletest",
        "googletest/include",
    ],
    linkopts = select({
        "@//tools:linux": ["-pthread"],
        "@//conditions:default": [],
    }),
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
