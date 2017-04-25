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
    name = "multibody_tree_context",
    srcs = [],
    hdrs = [
        "multibody_tree_context.h",
        "position_kinematics_cache.h",
    ],
    deps = [
        ":multibody_tree_indexes",
        "//drake/common:autodiff",
        "//drake/systems/framework:leaf_context",
    ],
)

drake_cc_library(
    name = "multibody_tree",
    srcs = [
        "fixed_offset_frame.cc",
        "multibody_tree.cc",
        "rigid_body.cc",
    ],
    hdrs = [
        "body.h",
        "fixed_offset_frame.h",
        "frame.h",
        "frame_base.h",
        "multibody_tree.h",
        "multibody_tree_element.h",
        "multibody_tree_topology.h",
        "rigid_body.h",
    ],
    deps = [
        ":multibody_tree_context",
        ":multibody_tree_indexes",
        ":spatial_inertia",
        "//drake/common:autodiff",
    ],
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
    name = "spatial_inertia",
    srcs = [],
    hdrs = ["spatial_inertia.h"],
    deps = [
        ":unit_inertia",
        "//drake/math:gradient",
        "//drake/multibody/multibody_tree/math:spatial_algebra",
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

# === test/ ===

drake_cc_googletest(
    name = "double_pendulum_test",
    deps = [":multibody_tree"],
)

drake_cc_googletest(
    name = "multibody_tree_creation_tests",
    deps = [":multibody_tree"],
)

drake_cc_googletest(
    name = "multibody_tree_indexes_tests",
    deps = [":multibody_tree_indexes"],
)

drake_cc_googletest(
    name = "rotational_inertia_tests",
    deps = [
        ":rotational_inertia",
        "//drake/common:autodiff",
    ],
)

drake_cc_googletest(
    name = "spatial_inertia_tests",
    deps = [
        ":spatial_inertia",
    ],
)

drake_cc_googletest(
    name = "unit_inertia_tests",
    deps = [
        ":unit_inertia",
    ],
)

cpplint()
