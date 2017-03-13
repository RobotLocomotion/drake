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
    deps = [
        "//drake/common",
        "//drake/thirdParty:spruce",
    ],
)

filegroup(
    name = "test_models",
    testonly = 1,
    srcs = [
        "test/colinear_vertex.obj",
        "test/concave_face_bad.obj",
        "test/concave_face_good.obj",
        "test/non_planar.obj",
        "test/out_of_order_vertex.obj",
        "test/quad_cube.obj",
        "test/tiny_triangle.obj",
        "test/tri_cube.obj",
    ],
)

drake_cc_googletest(
    name = "mesh_triangulate_test",
    srcs = ["test/mesh_triangulate_test.cc"],
    data = [
        ":test_models",
    ],
    deps = [
        ":shapes",
        "//drake/common:drake_path",
    ],
)

drake_cc_googletest(
    name = "face_query_test",
    srcs = ["test/face_query_test.cc"],
    data = [
        ":test_models",
    ],
    deps = [
        ":shapes",
        "//drake/common:drake_path",
    ],
)

drake_cc_googletest(
    name = "mesh_scaling_test",
    srcs = ["test/mesh_scaling_test.cc"],
    data = [
        ":test_models",
    ],
    deps = [
        ":shapes",
        "//drake/common:drake_path",
    ],
)

cpplint()
