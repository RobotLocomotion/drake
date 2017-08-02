# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
)

# === test/ ===

drake_cc_googletest(
    name = "dual_iiwa14_polytope_collision_test",
    srcs = ["urdf/test/dual_iiwa14_polytope_collision_test.cc"],
    data = [":models"],
    local = 1,
    deps = [
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
    ],
)

install_data()

add_lint_tests()
