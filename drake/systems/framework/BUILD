# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "framework",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
        "//drake/common:autodiff",
        "//drake/common:common",
        "//drake/common:number_traits",
    ],
    linkstatic=1)
