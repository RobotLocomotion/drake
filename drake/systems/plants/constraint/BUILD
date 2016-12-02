# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "direct_collocation_constraint",
    srcs = ["direct_collocation_constraint.cc"],
    hdrs = ["direct_collocation_constraint.h"],
    linkstatic = 1,
    deps = [
        "//drake/math:autodiff",
        "//drake/solvers:constraint",
        "//drake/systems/framework",
    ],
)
