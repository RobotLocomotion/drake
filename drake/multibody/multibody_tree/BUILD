# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "rotational_inertia",
    srcs = [],
    hdrs = ["rotational_inertia.h"],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:extract_double"
    ],
)

drake_cc_library(
    name = "unit_inertia",
    srcs = [],
    hdrs = ["unit_inertia.h"],
    deps = [
        ":rotational_inertia"
    ],
)

drake_cc_googletest(
    name = "mass_properties_test",
    deps = [
        ":rotational_inertia",
        "//drake/common:autodiff"
    ],
)

drake_cc_googletest(
    name = "unit_inertia_tests",
    deps = [
        ":unit_inertia"
    ],
)

cpplint()
