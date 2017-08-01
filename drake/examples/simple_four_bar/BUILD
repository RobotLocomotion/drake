# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
)

add_lint_tests()
