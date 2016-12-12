# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "cc_component", "cc_unit_test")

package(default_visibility = ["//visibility:public"])

cc_component(
    name = "gravity_compensator",
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
    ],
)

cc_component(
    name = "linear_quadratic_regulator",
    deps = [
        "//drake/common:is_approx_equal_abstol",
        "//drake/math:continuous_algebraic_ricatti_equation",
        "//drake/systems/framework",
        "//drake/systems/primitives:linear_system",
    ],
)

cc_component(
    name = "pid_controller",
    deps = [
        "//drake/systems/framework",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:pass_through",
    ],
)

cc_component(
    name = "pid_controlled_system",
    deps = [
        ":pid_controller",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:matrix_gain",
    ],
)

# === test/ ===

cc_unit_test(
    name = "gravity_compensator_test",
    data = [
        "//drake/examples/SimpleFourBar:models",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    deps = [
        ":gravity_compensator",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "linear_quadratic_regulator_test",
    deps = [
        ":linear_quadratic_regulator",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "pid_controlled_system_test",
    deps = [
        ":pid_controlled_system",
        "//drake/common:eigen_matrix_compare",
    ],
)

cc_unit_test(
    name = "pid_controller_test",
    deps = [
        ":pid_controller",
        "//drake/common:eigen_matrix_compare",
    ],
)
