# -*- python -*

package(default_visibility = ["//visibility:public"])

DEPS = ["//drake/common:common"]

cc_library(
    name = "util",
    srcs = [
        "drakeGeometryUtil.cpp",
        "drakeUtil.cpp",
    ],
    hdrs = [
        "drakeGeometryUtil.h",
        "drakeUtil.h",
    ],
    linkstatic = 1,
    deps = DEPS + [
        "//drake/math:geometric_transform",
        "//drake/math:gradient",
    ],
)
