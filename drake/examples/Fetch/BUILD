# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_googletest",
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.urdf",
    ]),
    visibility = ["//visibility:public"],
)

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "fetch_common",
    srcs = [
        "fetch_common.cc",
    ],
    hdrs = [
        "fetch_common.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "fetch_simulation_test",
    srcs = ["test/fetch_simulation_test.cc"],
    data = [
        "//drake/examples/Fetch:models",
    ],
    deps = [
        ":fetch_common",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

cpplint()
