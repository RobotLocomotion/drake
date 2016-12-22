# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parsers",
    srcs = [
        "model_instance_id_table.cc",
        "package_map.cc",
        "parser_common.cc",
        "sdf_parser.cc",
        "urdf_parser.cc",
        "xml_util.cc",
    ],
    hdrs = [
        "model_instance_id_table.h",
        "package_map.h",
        "parser_common.h",
        "sdf_parser.h",
        "urdf_parser.h",
        "xml_util.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_kinematics_test",
    testonly = 1,
    srcs = ["test/urdf_kinematics_test.cc"],
    linkstatic = 1,
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_collision_test",
    testonly = 1,
    srcs = ["test/urdf_collision_test.cc"],
    linkstatic = 1,
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
cc_binary(
    name = "urdf_manipulator_dynamics_test",
    srcs = ["test/urdf_manipulator_dynamics_test.cc"],
    linkstatic = 1,
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

cc_googletest(
    name = "package_map_test",
    srcs = ["test/package_map_test/package_map_test.cc"],
    data = [
        ":test_models",
    ],
    deps = [
        ":parsers",
    ],
)

cc_googletest(
    name = "urdf_parser_test",
    srcs = ["test/urdf_parser_test/urdf_parser_test.cc"],
    data = [
        ":test_models",
        "//drake/examples/Atlas:models",
    ],
    deps = [
        ":parsers",
    ],
)

cc_googletest(
    name = "xml_util_test",
    deps = [
        ":parsers",
    ],
)

filegroup(
    name = "test_models",
    testonly = 1,
    srcs = glob([
        "test/**/*.obj",
        "test/**/*.sdf",
        "test/**/*.urdf",
        "test/**/*.xml",
    ]),
)
