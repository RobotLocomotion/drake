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
    name = "multibody_tree_indexes",
    srcs = [],
    hdrs = ["multibody_tree_indexes.h"],
    deps = [
        "//drake/common",
        "//drake/common:nice_type_name",
    ],
)

drake_cc_library(
    name = "rotational_inertia",
    srcs = [],
    hdrs = ["rotational_inertia.h"],
    deps = [
        "//drake/common",
        "//drake/common:extract_double"
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
    name = "multibody_tree_indexes_tests",
    deps = [":multibody_tree_indexes"],
)

cpplint()
