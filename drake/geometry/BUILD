# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

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
        "//drake/common",
    ],
)

drake_cc_library(
    name = "geometry_query_results",
    srcs = [],
    hdrs = ["geometry_query_results.h"],
    deps = [
        ":geometry_ids",
        "//drake/common",
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
        ":geometry_query_results",
        "//drake/common",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "identifier",
    srcs = [],
    hdrs = ["identifier.h"],
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

cpplint()
