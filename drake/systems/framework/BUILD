# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "framework",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:number_traits",
    ],
)
