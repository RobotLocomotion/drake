# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_utilities",
    testonly = 1,
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
    ],
)
