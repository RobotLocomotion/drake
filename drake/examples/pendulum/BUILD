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
    name = "pendulum_vector_types",
    srcs = [
        "gen/pendulum_input.cc",
        "gen/pendulum_params.cc",
        "gen/pendulum_state.cc",
    ],
    hdrs = [
        "gen/pendulum_input.h",
        "gen/pendulum_params.h",
        "gen/pendulum_state.h",
    ],
    deps = [
        "//drake/systems/framework:vector",
    ],
)

drake_cc_library(
    name = "pendulum_plant",
    srcs = ["pendulum_plant.cc"],
    hdrs = ["pendulum_plant.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":pendulum_vector_types",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "passive_simulation",
    srcs = ["passive_simulation.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--target_realtime_rate=0.0"],
    deps = [
        ":pendulum_plant",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "energy_shaping_simulation",
    srcs = ["energy_shaping_simulation.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--target_realtime_rate=0.0"],
    deps = [
        ":pendulum_plant",
        "//drake/common:find_resource",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:leaf_system",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "lqr_simulation",
    srcs = ["lqr_simulation.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--target_realtime_rate=0.0"],
    deps = [
        ":pendulum_plant",
        "//drake/common:find_resource",
        "//drake/common:is_approx_equal_abstol",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:linear_quadratic_regulator",
        "//drake/systems/framework:diagram",
        "//drake/systems/framework:leaf_system",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "trajectory_optimization_simulation",
    srcs = ["trajectory_optimization_simulation.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--target_realtime_rate=0.0"],
    deps = [
        "pendulum_plant",
        "//drake/common:find_resource",
        "//drake/common:is_approx_equal_abstol",
        "//drake/lcm",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:pid_controlled_system",
        "//drake/systems/framework",
        "//drake/systems/primitives:trajectory_source",
        "//drake/systems/trajectory_optimization:direct_collocation",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "print_symbolic_dynamics",
    srcs = ["print_symbolic_dynamics.cc"],
    add_test_rule = 1,
    deps = [
        ":pendulum_plant",
        "//drake/common:symbolic",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "urdf_dynamics_test",
    data = ["Pendulum.urdf"],
    deps = [
        ":pendulum_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "pendulum_plant_test",
    deps = [
        ":pendulum_plant",
        "//drake/common:autodiff",
    ],
)

install_data()

add_lint_tests()
