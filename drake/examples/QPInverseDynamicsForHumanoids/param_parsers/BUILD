# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_library", "drake_cc_googletest")
load("@protobuf//:protobuf.bzl", "cc_proto_library")

package(
    default_visibility =
        ["//drake/examples/QPInverseDynamicsForHumanoids:__subpackages__"],
)

drake_cc_library(
    name = "param_parser",
    srcs = [
        "param_parser.cc",
    ],
    hdrs = [
        "param_parser.h",
    ],
    deps = [
        ":id_controller_config",
        ":rigid_body_tree_alias_groups",
        "//drake/common:protobuf",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller_common",
        "@yaml_cpp//:lib",
    ],
)

cc_proto_library(
    name = "alias_groups",
    srcs = [
        "alias_groups.proto",
    ],
)

cc_proto_library(
    name = "id_controller_config",
    srcs = [
        "id_controller_config.proto",
    ],
)

drake_cc_library(
    name = "rigid_body_tree_alias_groups",
    srcs = ["rigid_body_tree_alias_groups.cc"],
    hdrs = ["rigid_body_tree_alias_groups.h"],
    deps = [
        ":alias_groups",
        "//drake/common:protobuf",
        "//drake/multibody:rigid_body_tree",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "rigid_body_tree_alias_groups_test",
    srcs = ["test/rigid_body_tree_alias_groups_test.cc"],
    data = [
        "test/test.alias_groups",
        "//drake/multibody:test_models",
    ],
    deps = [
        ":rigid_body_tree_alias_groups",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "param_parser_test",
    srcs = ["test/param_parser_test.cc"],
    data = [
        "test/params.alias_groups",
        "test/params.id_controller_config",
        "//drake/examples/Valkyrie:models",
    ],
    deps = [
        ":param_parser",
        ":rigid_body_tree_alias_groups",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

cpplint()
