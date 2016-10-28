# -*- python -*-

cc_library(
    name = "framework",
    hdrs = glob(["framework/*.h"]),
    srcs = glob(["framework/*.cc"]),
    visibility = ["//visibility:public"],
    deps = [
        "//drake/common:common",
        "//drake/common:symbolic",
        "//drake/math:autodiff_overloads",
    ],
)

cc_library(
    name = "analysis",
    hdrs = glob(["analysis/*.h"]),
    srcs = glob(["analysis/*.cc"]),
    visibility = ["//visibility:public"],
    deps = [":framework"],
)
