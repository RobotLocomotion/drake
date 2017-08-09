# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "geometry_frame",
    srcs = [],
    hdrs = ["geometry_frame.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "geometry_ids",
    srcs = [],
    hdrs = ["geometry_ids.h"],
    deps = [":identifier"],
)

drake_cc_library(
    name = "geometry_instance",
    srcs = ["geometry_instance.cc"],
    hdrs = ["geometry_instance.h"],
    deps = [
        ":shape_specification",
        "//drake/common",
        "//drake/common:copyable_unique_ptr",
    ],
)

drake_cc_library(
    name = "geometry_state",
    srcs = ["geometry_state.cc"],
    hdrs = ["geometry_state.h"],
    deps = [
        ":geometry_frame",
        ":geometry_ids",
        ":geometry_instance",
        ":internal_frame",
        ":internal_geometry",
    ],
)

drake_cc_library(
    name = "geometry_system",
    srcs = ["geometry_system.cc"],
    hdrs = [
        "geometry_system.h",
        "query_handle.h",
    ],
    deps = [
        ":geometry_frame",
        ":geometry_instance",
        ":geometry_state",
        "//drake/common",
        "//drake/geometry/query_results:penetration_as_point_pair",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "internal_frame",
    srcs = ["internal_frame.cc"],
    hdrs = ["internal_frame.h"],
    deps = [
        ":geometry_ids",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "internal_geometry",
    srcs = [],
    hdrs = ["internal_geometry.h"],
    deps = [
        ":geometry_ids",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
    deps = ["//drake/common"],
)

drake_cc_library(
    name = "shape_specification",
    srcs = [],
    hdrs = ["shape_specification.h"],
    deps = ["//drake/common"],
)

# -----------------------------------------------------

drake_cc_library(
    name = "expect_error_message",
    srcs = [],
    hdrs = ["test/expect_error_message.h"],
)

drake_cc_googletest(
    name = "geometry_system_test",
    deps = [
        ":expect_error_message",
        ":geometry_system",
    ],
)

drake_cc_googletest(
    name = "identifier_test",
    deps = [":identifier"],
)

add_lint_tests()
