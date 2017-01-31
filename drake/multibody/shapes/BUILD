# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "shapes",
    srcs = [
        "element.cc",
        "geometry.cc",
        "visual_element.cc",
    ],
    hdrs = [
        "drake_shapes.h",
        "element.h",
        "geometry.h",
        "visual_element.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/thirdParty:spruce",
    ],
)

drake_cc_googletest(
    name = "mesh_triangulate_test",
    srcs = ["test/mesh_triangulate_test.cc"],
    data = [
        "test/colinear_vertex.obj",
        "test/concave_face_bad.obj",
        "test/concave_face_good.obj",
        "test/non_planar.obj",
        "test/out_of_order_vertex.obj",
        "test/quad_cube.obj",
        "test/tiny_triangle.obj",
        "test/tri_cube.obj",
    ],
    deps = [
        ":shapes",
        "//drake/common:drake_path",
    ],
)

cpplint()
