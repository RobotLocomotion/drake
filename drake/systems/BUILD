# -*- python -*-

cc_library(
    name = "systems_framework",
    hdrs = glob(["framework/*.h"]),
    srcs = glob(["framework/*.cc"]),
    visibility = ["//visibility:public"],
    deps = ["//drake/math"],
)

cc_library(
    name = "systems_analysis",
    hdrs = glob(["analysis/*.h"]),
    srcs = glob(["analysis/*.cc"]),
    visibility = ["//visibility:public"],
    deps = ["systems_framework"],
)
