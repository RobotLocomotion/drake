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
        "//drake/common:type_safe_index",
    ],
)

drake_cc_library(
    name = "multibody_tree",
    srcs = [
        "multibody_tree.cc",
        "rigid_body.cc",
    ],
    hdrs = [
        "body.h",
        "multibody_tree.h",
        "multibody_tree_element.h",
        "rigid_body.h",
    ],
    deps = [
        ":multibody_tree_indexes",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "rotational_inertia",
    srcs = [],
    hdrs = ["rotational_inertia.h"],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

drake_cc_library(
    name = "unit_inertia",
    srcs = [],
    hdrs = ["unit_inertia.h"],
    deps = [
        ":rotational_inertia",
    ],
)

drake_cc_googletest(
    name = "multibody_tree_indexes_tests",
    deps = [":multibody_tree_indexes"],
)

drake_cc_googletest(
    name = "multibody_tree_creation_tests",
    deps = [":multibody_tree"],
)

drake_cc_googletest(
    name = "rotational_inertia_tests",
    deps = [
        ":rotational_inertia",
        "//drake/common:autodiff",
    ],
)

drake_cc_googletest(
    name = "unit_inertia_tests",
    deps = [
        ":unit_inertia",
    ],
)

cpplint()
