# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_binary")
load("//tools:lint.bzl", "add_lint_tests")
load("@drake//tools:install.bzl", "install")

exports_files([
    "CPPLINT.cfg",
])

drake_cc_binary(
    name = "simple_continuous_time_system",
    srcs = ["simple_continuous_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:vector_system",
    ],
)

drake_cc_binary(
    name = "simple_discrete_time_system",
    srcs = ["simple_discrete_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:vector_system",
    ],
)

drake_cc_binary(
    name = "simple_mixed_continuous_and_discrete_time_system",
    srcs = ["simple_mixed_continuous_and_discrete_time_system.cc"],
    add_test_rule = 1,
    deps = [
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:leaf_system",
    ],
)

# These are installed as part of the Drake release process, and loaded into
# drake_visualizer's model database by default.
PUBLISHED_MODELS = [
    "//drake/examples/acrobot",
    "//drake/examples/atlas",
    "//drake/examples/contact_model",
    "//drake/examples/double_pendulum",
    "//drake/examples/irb140",
    "//drake/examples/kuka_iiwa_arm",
    "//drake/examples/particles",
    "//drake/examples/pendulum",
    "//drake/examples/pr2",
    "//drake/examples/quadrotor",
    "//drake/examples/simple_four_bar",
    "//drake/examples/valkyrie",
    "//drake/examples/zmp",
]

filegroup(
    name = "prod_models",
    data = [dirname + ":prod_models" for dirname in PUBLISHED_MODELS],
    visibility = ["//visibility:public"],
)

install(
    name = "install_data",
    visibility = ["//visibility:public"],
    deps = [dirname + ":install_data" for dirname in PUBLISHED_MODELS],
)

add_lint_tests()
