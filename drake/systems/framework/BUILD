# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "framework",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
        "//drake/common:common",
        "//drake/common:symbolic",
        "//drake/math:autodiff_overloads",
    ],
    linkstatic=1)
