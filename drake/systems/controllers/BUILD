# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gravity_compensator",
    srcs = ["gravity_compensator.cc"],
    hdrs = ["gravity_compensator.h"],
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers:multi_body_parsers",
        "//drake/systems/framework",
    ],
)

cc_library(
    name = "linear_quadratic_regulator",
    srcs = ["linear_quadratic_regulator.cc"],
    hdrs = ["linear_quadratic_regulator.h"],
    linkstatic = 1,
    deps = [
        "//drake/common:is_approx_equal_abstol",
        "//drake/math:continuous_algebraic_ricatti_equation",
        "//drake/systems/framework",
        "//drake/systems/primitives:linear_system",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    linkstatic = 1,
    deps = [
        "//drake/systems/framework",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:pass_through",
    ],
)

cc_library(
    name = "pid_controlled_system",
    srcs = ["pid_controlled_system.cc"],
    hdrs = ["pid_controlled_system.h"],
    linkstatic = 1,
    deps = [
        ":pid_controller",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:matrix_gain",
    ],
)

# === test/ ===

cc_test(
    name = "gravity_compensator_test",
    size = "small",
    srcs = ["test/gravity_compensator_test.cc"],
    data = [
        "//drake/examples/SimpleFourBar:models",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    linkstatic = 1,
    deps = [
        ":gravity_compensator",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "linear_quadratic_regulator_test",
    size = "small",
    srcs = ["test/linear_quadratic_regulator_test.cc"],
    linkstatic = 1,
    deps = [
        ":linear_quadratic_regulator",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pid_controlled_system_test",
    size = "small",
    srcs = ["test/pid_controlled_system_test.cc"],
    linkstatic = 1,
    deps = [
        ":pid_controlled_system",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["test/pid_controller_test.cc"],
    linkstatic = 1,
    deps = [
        ":pid_controller",
        "//drake/common:eigen_matrix_compare",
        "@gtest//:main",
    ],
)
