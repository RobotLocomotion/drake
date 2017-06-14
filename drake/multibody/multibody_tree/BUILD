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
    name = "multibody_tree_topology",
    srcs = [],
    hdrs = [
        "multibody_tree_topology.h",
    ],
    deps = [
        ":multibody_tree_indexes",
    ],
)

drake_cc_library(
    name = "multibody_tree_context",
    srcs = [
        "multibody_tree_context.cc",
        "position_kinematics_cache.cc",
    ],
    hdrs = [
        "multibody_tree_context.h",
        "position_kinematics_cache.h",
    ],
    deps = [
        ":multibody_tree_topology",
        "//drake/common:autodiff",
        "//drake/systems/framework:leaf_context",
    ],
)

drake_cc_library(
    name = "multibody_tree_element",
    srcs = [],
    hdrs = [
        "multibody_tree_element.h",
    ],
    deps = [
        ":multibody_tree_topology",
    ],
)

drake_cc_library(
    name = "bodies_and_frames",
    srcs = [
        "fixed_offset_frame.cc",
        "rigid_body.cc",
    ],
    hdrs = [
        "body.h",
        "fixed_offset_frame.h",
        "frame.h",
        "frame_base.h",
        "rigid_body.h",
    ],
    deps = [
        ":multibody_tree_context",
        ":multibody_tree_element",
        ":spatial_inertia",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "mobilizers_and_body_nodes",
    srcs = [
        "body_node_impl.cc",
        "mobilizer_impl.cc",
        "revolute_mobilizer.cc",
    ],
    hdrs = [
        "body_node.h",
        "body_node_impl.h",
        "body_node_welded.h",
        "mobilizer.h",
        "mobilizer_impl.h",
        "revolute_mobilizer.h",
    ],
    deps = [
        ":bodies_and_frames",
        ":multibody_tree_context",
        "//drake/common:autodiff",
        "//drake/common:type_safe_index",
    ],
)

drake_cc_library(
    name = "multibody_tree",
    srcs = [
        "multibody_tree.cc",
    ],
    hdrs = [
        "multibody_tree.h",
    ],
    deps = [
        ":bodies_and_frames",
        ":mobilizers_and_body_nodes",
        ":multibody_tree_context",
        ":multibody_tree_indexes",
        ":spatial_inertia",
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
    deps = [
        ":multibody_tree",
        "//drake/multibody/benchmarks/acrobot",
    ],
)

drake_cc_googletest(
    name = "frames_test",
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
