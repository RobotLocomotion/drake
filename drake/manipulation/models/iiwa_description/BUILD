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
    name = "dual_iiwa14_polytope_collision_test",
    srcs = ["urdf/test/dual_iiwa14_polytope_collision_test.cc"],
    data = [":models"],
    local = 1,
    deps = [
        "//drake/multibody/parsers",
    ],
)

cpplint()
