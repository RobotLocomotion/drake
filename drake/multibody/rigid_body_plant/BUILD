# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rigid_body_plant",
    srcs = [
        "contact_detail.cc",
        "contact_force.cc",
        "contact_info.cc",
        "contact_resultant_force_calculator.cc",
        "contact_results.cc",
        "kinematics_results.cc",
        "point_contact_detail.cc",
        "rigid_body_plant.cc",
    ],
    hdrs = [
        "contact_detail.h",
        "contact_force.h",
        "contact_info.h",
        "contact_resultant_force_calculator.h",
        "contact_results.h",
        "kinematics_results.h",
        "point_contact_detail.h",
        "rigid_body_plant.h",
    ],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "contact_results_to_lcm",
    srcs = [
        "contact_results_to_lcm.cc",
    ],
    hdrs = [
        "contact_results_to_lcm.h",
    ],
    deps = [
        ":rigid_body_plant",
        "//drake/lcmtypes:contact_info_for_viz",
        "//drake/lcmtypes:contact_results_for_viz",
    ],
)

drake_cc_library(
    name = "create_load_robot_message",
    srcs = ["create_load_robot_message.cc"],
    hdrs = ["create_load_robot_message.h"],
    deps = [
        "//drake/lcmtypes:viewer",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/rendering:drake_visualizer_client",
    ],
)

drake_cc_library(
    name = "rigid_body_plant_that_publishes_xdot",
    srcs = [
        "rigid_body_plant_that_publishes_xdot.cc",
    ],
    hdrs = [
        "rigid_body_plant_that_publishes_xdot.h",
    ],
    deps = [
        ":rigid_body_plant",
        "//drake/lcm",
        "//drake/lcmtypes:drake_signal",
    ],
)

drake_cc_library(
    name = "drake_visualizer",
    srcs = [
        "drake_visualizer.cc",
        "viewer_draw_translator.cc",
    ],
    hdrs = [
        "drake_visualizer.h",
        "viewer_draw_translator.h",
    ],
    deps = [
        ":create_load_robot_message",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/lcmtypes:viewer",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/lcm",
        "//drake/systems/primitives:signal_log",
        "//drake/systems/rendering:drake_visualizer_client",
    ],
)

drake_cc_googletest(
    name = "compute_contact_result_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "contact_detail_test",
    deps = [
        ":rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "contact_force_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "contact_force_formula_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "contact_info_test",
    deps = [
        ":rigid_body_plant",
    ],
)

drake_cc_googletest(
    name = "contact_resultant_force_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "drake_visualizer_test",
    data = ["//drake/multibody/collision:test_models"],
    deps = [
        ":drake_visualizer",
        "//drake/common:drake_path",
        "//drake/lcm:mock",
        "//drake/systems/analysis",
    ],
)

drake_cc_googletest(
    name = "kinematics_results_test",
    data = ["//drake/multibody:test_models"],
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_plant_test",
    data = [
        ":test_models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/multibody:models",
        "//drake/multibody:test_models",
    ],
    deps = [
        ":rigid_body_plant",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "rigid_body_plant_that_publishes_xdot_test",
    data = [
        "//drake/multibody:models",
    ],
    deps = [
        ":rigid_body_plant_that_publishes_xdot",
        "//drake/common:drake_path",
        "//drake/lcm:lcmt_drake_signal_utils",
        "//drake/lcm:mock",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "viewer_draw_translator_test",
    deps = [
        ":drake_visualizer",
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
