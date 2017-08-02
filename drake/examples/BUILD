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

install(
    name = "install_data",
    visibility = ["//visibility:public"],
    deps = [
        "//drake/examples/acrobot:install_data",
        "//drake/examples/atlas:install_data",
        "//drake/examples/contact_model:install_data",
        "//drake/examples/double_pendulum:install_data",
        "//drake/examples/irb140:install_data",
        "//drake/examples/kuka_iiwa_arm:install_data",
        "//drake/examples/particles:install_data",
        "//drake/examples/pendulum:install_data",
        "//drake/examples/pr2:install_data",
        "//drake/examples/quadrotor:install_data",
        "//drake/examples/simple_four_bar:install_data",
        "//drake/examples/valkyrie:install_data",
        "//drake/examples/zmp:install_data",
    ],
)

add_lint_tests()
