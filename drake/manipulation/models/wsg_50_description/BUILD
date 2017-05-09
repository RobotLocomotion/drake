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
    name = "wsg_50_model_files",
    srcs = [
        "meshes/GUIDE_WSG50_110.obj",
        "meshes/WSG-FMF.obj",
        "meshes/WSG50_110.obj",
        "urdf/wsg_50_mesh_collision.urdf",
        "package.xml",
    ],
    visibility = ["//visibility:public"],
)

# === test/ ===

drake_cc_googletest(
    name = "wsg_50_mesh_collision_test",
    srcs = ["urdf/test/wsg50_mesh_collision_test.cc"],
    data = [":wsg_50_model_files"],
    local = 1,
    deps = [
        "//drake/multibody/parsers",
    ],
)

cpplint()
