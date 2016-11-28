# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "analysis",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = ["//drake/systems/framework"],
    linkstatic = 1)
