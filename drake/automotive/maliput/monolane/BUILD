# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monolane",
    srcs = [],
    hdrs = [],
    linkstatic = 1,
    deps = [
        ":builder",
        ":lanes",
        ":loader",
    ],
)

cc_library(
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
    linkstatic = 1,
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

cc_library(
    name = "builder",
    srcs = [
        "builder.cc",
    ],
    hdrs = [
        "builder.h",
    ],
    linkstatic = 1,
    deps = [
        ":lanes",
    ],
)

cc_library(
    name = "loader",
    srcs = [
        "loader.cc",
    ],
    hdrs = [
        "loader.h",
    ],
    linkstatic = 1,
    deps = [
        ":builder",
        "@yaml_cpp//:lib",
    ],
)

filegroup(
    name = "yamls",
    srcs = glob(["*.yaml"]),
)

# === test/ ===

cc_test(
    name = "monolane_builder_test",
    size = "small",
    srcs = ["test/monolane_builder_test.cc"],
    deps = [
        ":builder",
        "@gtest//:main",
    ],
)

cc_test(
    name = "monolane_lanes_test",
    size = "small",
    srcs = ["test/monolane_lanes_test.cc"],
    deps = [
        ":lanes",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_binary(
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
