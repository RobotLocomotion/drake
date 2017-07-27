# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)
load("//tools:install_data.bzl", "install_data")

package(
    default_visibility = [":__subpackages__"],
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

install_data()

cpplint()
