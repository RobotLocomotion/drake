# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "monolane",
    srcs = [],
    hdrs = [],
    deps = [
        ":builder",
        ":lanes",
        ":loader",
        ":road_section_builder",
    ],
)

drake_cc_library(
    name = "lanes",
    srcs = [
        "arc_lane.cc",
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "line_lane.cc",
        "road_geometry.cc",
        "segment.cc",
    ],
    hdrs = [
        "arc_lane.h",
        "branch_point.h",
        "junction.h",
        "lane.h",
        "line_lane.h",
        "road_geometry.h",
        "segment.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

drake_cc_library(
    name = "builder",
    srcs = [
        "builder.cc",
    ],
    hdrs = [
        "builder.h",
    ],
    deps = [
        ":lanes",
    ],
)

drake_cc_library(
    name = "loader",
    srcs = [
        "loader.cc",
    ],
    hdrs = [
        "loader.h",
    ],
    deps = [
        ":builder",
        "@yaml_cpp//:lib",
    ],
)

drake_cc_library(
    name = "road_section_builder",
    srcs = [
        "road_section_builder.cc",
    ],
    hdrs = [
        "road_section_builder.h",
    ],
    linkstatic = 1,
    deps = [
        ":builder",
    ],
)

filegroup(
    name = "yamls",
    srcs = glob(["*.yaml"]),
)

# === test/ ===

drake_cc_googletest(
    name = "monolane_builder_test",
    size = "small",
    srcs = ["test/monolane_builder_test.cc"],
    deps = [
        ":builder",
    ],
)

drake_cc_googletest(
    name = "monolane_lanes_test",
    size = "small",
    srcs = ["test/monolane_lanes_test.cc"],
    deps = [
        ":lanes",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_binary(
    name = "yaml_load",
    testonly = 1,
    srcs = ["test/yaml_load.cc"],
    deps = [
        ":loader",
        "//drake/common:text_logging_gflags",
    ],
)

py_test(
    name = "yaml_load_test",
    size = "small",
    srcs = ["test/yaml_load_test.py"],
    args = ["drake/automotive/maliput/monolane/yaml_load"],
    data = [
        ":yaml_load",
        ":yamls",
    ],
)

cpplint()
