# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "cosserat_rod_plant",
    srcs = [
        "cosserat_rod_plant.cc",
        "rod_element.cc",
    ],
    hdrs = [
        "cosserat_rod_plant.h",
        "rod_element.h",
    ],
    deps = [
        "//drake/lcmtypes:viewer",
        "//drake/multibody/multibody_tree",
        "//drake/systems/framework",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_bundle",
    ],
)

drake_cc_binary(
    name = "cantilever_run",
    srcs = ["cantilever_run.cc"],
    add_test_rule = 1,
    data = ["//drake/examples/acrobot:models"],
    test_rule_args = ["-realtime_factor=0.0"],
    deps = [
        ":cosserat_rod_plant",
        "//drake/lcm",
        "//drake/lcmtypes:viewer",
        "//drake/systems/analysis:implicit_euler_integrator",
        "//drake/systems/analysis:runge_kutta2_integrator",
        "//drake/systems/analysis:semi_explicit_euler_integrator",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/lcm",
        "//drake/systems/primitives:signal_logger",
        "//drake/systems/rendering:drake_visualizer_client",
        "//drake/systems/rendering:pose_aggregator",
        "//drake/systems/rendering:pose_bundle_to_draw_message",
        "@com_github_gflags_gflags//:gflags",
    ],
)

#drake_cc_googletest(
#    name = "acrobot_multibody_plant_test",
#    deps = [
#        ":acrobot_multibody_plant",
#        "//drake/multibody/benchmarks/acrobot",
#    ],
#)

install_data()

add_lint_tests()
