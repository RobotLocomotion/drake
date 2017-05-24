# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)

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
    local = 1,
    deps = [
        "//drake/multibody/parsers",
    ],
)

cpplint()
