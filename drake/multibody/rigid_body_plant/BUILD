# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_googletest")

package(default_visibility = ["//visibility:public"])

cc_library(
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
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/solvers:mathematical_program",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "drake_visualizer",
    srcs = [
        "drake_visualizer.cc",
        "viewer_draw_translator.cc",
    ],
    hdrs = [
        "drake_visualizer.h",
        "viewer_draw_translator.h",
    ],
    linkstatic = 1,
    deps = [
        "//drake/lcmtypes:viewer",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/lcm",
    ],
)

cc_googletest(
    name = "compute_contact_result_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "contact_detail_test",
    deps = [
        ":rigid_body_plant",
    ],
)

cc_googletest(
    name = "contact_force_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "contact_info_test",
    deps = [
        ":rigid_body_plant",
    ],
)

cc_googletest(
    name = "contact_resultant_force_test",
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_googletest(
    name = "drake_visualizer_test",
    data = ["//drake/multibody/collision:test_models"],
    deps = [
        ":drake_visualizer",
        "//drake/lcm:mock",
    ],
)

cc_googletest(
    name = "kinematics_results_test",
    data = ["//drake/multibody:test_models"],
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

cc_googletest(
    name = "rigid_body_plant_test",
    data = [
        ":test_models",
        "//drake/examples/kuka_iiwa_arm:models",
        "//drake/multibody:test_models",
    ],
    deps = [
        ":rigid_body_plant",
        "//drake/common:drake_path",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

cc_googletest(
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
