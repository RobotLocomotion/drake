# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
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
    deps = [
        "//drake/common:drake_path",
        "//drake/multibody:rigid_body_tree",
        "//drake/thirdParty:spruce",
        "//drake/thirdParty:tinydir",
        "//drake/thirdParty:tinyxml2",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
drake_cc_binary(
    name = "urdf_kinematics_test",
    testonly = 1,
    srcs = ["test/urdf_kinematics_test.cc"],
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
drake_cc_binary(
    name = "urdf_collision_test",
    testonly = 1,
    srcs = ["test/urdf_collision_test.cc"],
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

# TODO(jwnimmer-tri) This is just some random program.  Do we want to keep it?
drake_cc_binary(
    name = "urdf_manipulator_dynamics_test",
    srcs = ["test/urdf_manipulator_dynamics_test.cc"],
    deps = [
        ":parsers",
        "//drake/multibody:rigid_body_tree",
        "@gtest//:main",
    ],
)

drake_cc_googletest(
    name = "package_map_test",
    srcs = ["test/package_map_test/package_map_test.cc"],
    data = [
        ":test_models",
    ],
    deps = [
        ":parsers",
    ],
)

drake_cc_googletest(
    name = "parser_common_test",
    srcs = ["test/parser_common_test/parser_common_test.cc"],
    data = ["test/parser_common_test/test_file.txt"],
    deps = [
        ":parsers",
    ],
)

drake_cc_googletest(
    name = "parsers_frames_test",
    srcs = ["test/parsers_frames_test/parsers_frames_test.cc"],
    data = [":test_models"],
    deps = [
        ":parsers",
    ],
)

drake_cc_googletest(
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

drake_cc_googletest(
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

cpplint()
