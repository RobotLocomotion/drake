# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:lint.bzl", "add_lint_tests")

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
        "acceleration_kinematics_cache.cc",
        "multibody_tree_context.cc",
        "position_kinematics_cache.cc",
        "velocity_kinematics_cache.cc",
    ],
    hdrs = [
        "acceleration_kinematics_cache.h",
        "multibody_tree_context.h",
        "position_kinematics_cache.h",
        "velocity_kinematics_cache.h",
    ],
    deps = [
        ":multibody_tree_topology",
        "//drake/common:autodiff",
        "//drake/multibody/multibody_tree/math:spatial_acceleration",
        "//drake/multibody/multibody_tree/math:spatial_velocity",
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
    name = "multibody_tree",
    srcs = [
        "body.cc",
        "body_node_impl.cc",
        "fixed_offset_frame.cc",
        "mobilizer_impl.cc",
        "multibody_tree.cc",
        "revolute_mobilizer.cc",
        "rigid_body.cc",
    ],
    hdrs = [
        "body.h",
        "body_node.h",
        "body_node_impl.h",
        "body_node_welded.h",
        "fixed_offset_frame.h",
        "frame.h",
        "frame_base.h",
        "mobilizer.h",
        "mobilizer_impl.h",
        "multibody_tree.h",
        "revolute_mobilizer.h",
        "rigid_body.h",
    ],
    deps = [
        ":multibody_tree_context",
        ":multibody_tree_element",
        ":multibody_tree_indexes",
        ":spatial_inertia",
        "//drake/common:autodiff",
    ],
)

drake_cc_library(
    name = "rotational_inertia",
    srcs = ["rotational_inertia.cc"],
    hdrs = ["rotational_inertia.h"],
    deps = [
        "//drake/common",
        "//drake/common:autodiff",
        "//drake/common:extract_double",
    ],
)

drake_cc_library(
    name = "spatial_inertia",
    srcs = ["spatial_inertia.cc"],
    hdrs = ["spatial_inertia.h"],
    deps = [
        ":unit_inertia",
        "//drake/math:gradient",
        "//drake/multibody/multibody_tree/math:spatial_algebra",
    ],
)

drake_cc_library(
    name = "unit_inertia",
    srcs = ["unit_inertia.cc"],
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
    name = "multibody_tree_creation_test",
    deps = [":multibody_tree"],
)

drake_cc_googletest(
    name = "multibody_tree_indexes_test",
    deps = [":multibody_tree_indexes"],
)

drake_cc_googletest(
    name = "rotational_inertia_test",
    deps = [
        ":rotational_inertia",
        "//drake/common:autodiff",
        "//drake/math:gradient",
    ],
)

drake_cc_googletest(
    name = "spatial_inertia_test",
    deps = [
        ":spatial_inertia",
    ],
)

drake_cc_googletest(
    name = "unit_inertia_test",
    deps = [
        ":unit_inertia",
        "//drake/math:gradient",
    ],
)

add_lint_tests()
