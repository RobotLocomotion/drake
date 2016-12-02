# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

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

cc_test(
    name = "contact_detail_test",
    size = "small",
    srcs = ["test/contact_detail_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_plant",
        "@gtest//:main",
    ],
)

cc_test(
    name = "contact_info_test",
    size = "small",
    srcs = ["test/contact_info_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_plant",
        "@gtest//:main",
    ],
)

cc_test(
    name = "contact_force_test",
    size = "small",
    srcs = ["test/contact_force_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "contact_resultant_force_test",
    size = "small",
    srcs = ["test/contact_resultant_force_test.cc"],
    linkstatic = 1,
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "drake_visualizer_test",
    size = "small",
    srcs = ["test/drake_visualizer_test.cc"],
    data = ["//drake/multibody:test_models"],
    linkstatic = 1,
    deps = [
        ":drake_visualizer",
        "//drake/lcm:mock",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kinematics_results_test",
    size = "small",
    srcs = ["test/kinematics_results_test.cc"],
    data = ["//drake/multibody:test_models"],
    linkstatic = 1,
    deps = [
        ":rigid_body_plant",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "viewer_draw_translator_test",
    size = "small",
    srcs = ["test/viewer_draw_translator_test.cc"],
    linkstatic = 1,
    deps = [
        ":drake_visualizer",
        "@gtest//:main",
    ],
)
