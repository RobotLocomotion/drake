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
    srcs = [
        "meshes/GUIDE_WSG50_110.obj",
        "meshes/WSG-FMF.obj",
        "meshes/WSG50_110.obj",
        "package.xml",
        "sdf/schunk_wsg_50.sdf",
        "sdf/schunk_wsg_50_ball_contact.sdf",
        "urdf/wsg_50_mesh_collision.urdf",
    ],
    visibility = ["//visibility:public"],
)

# === test/ ===

drake_cc_googletest(
    name = "wsg_50_mesh_collision_test",
    srcs = ["urdf/test/wsg50_mesh_collision_test.cc"],
    data = [":models"],
    local = 1,
    deps = [
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
    ],
)

add_lint_tests()
