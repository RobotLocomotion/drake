# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "analysis",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    deps = [
        "//drake/common:extract_double",
        "//drake/systems/framework",
    ],
)
