# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "analysis",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = ["//drake/systems/framework"],
    linkstatic = 1)
