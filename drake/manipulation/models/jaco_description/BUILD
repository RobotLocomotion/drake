# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

# === test/ ===

drake_cc_googletest(
    name = "jaco_arm_test",
    srcs = ["urdf/test/jaco_arm_test.cc"],
    data = [":models"],
    deps = [
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
    ],
)

add_lint_tests()
