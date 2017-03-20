# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "state_feedback_controller_base",
    hdrs = ["state_feedback_controller_base.h"],
    deps = [
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "model_based_controller_base",
    hdrs = ["model_based_controller_base.h"],
    deps = [
        ":state_feedback_controller_base",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "inverse_dynamics",
    srcs = ["inverse_dynamics.cc"],
    hdrs = ["inverse_dynamics.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "inverse_dynamics_controller",
    srcs = ["inverse_dynamics_controller.cc"],
    hdrs = ["inverse_dynamics_controller.h"],
    deps = [
        ":inverse_dynamics",
        ":model_based_controller_base",
        ":pid_controller",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_library(
    name = "linear_quadratic_regulator",
    srcs = ["linear_quadratic_regulator.cc"],
    hdrs = ["linear_quadratic_regulator.h"],
    deps = [
        "//drake/common:is_approx_equal_abstol",
        "//drake/math:continuous_algebraic_ricatti_equation",
        "//drake/systems/framework",
        "//drake/systems/primitives:linear_system",
    ],
)

drake_cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    deps = [
        ":state_feedback_controller_base",
        "//drake/systems/framework",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:gain",
        "//drake/systems/primitives:integrator",
        "//drake/systems/primitives:matrix_gain",
        "//drake/systems/primitives:pass_through",
        "//drake/systems/primitives:saturation",
    ],
)

drake_cc_library(
    name = "pid_controlled_system",
    srcs = ["pid_controlled_system.cc"],
    hdrs = ["pid_controlled_system.h"],
    deps = [
        ":pid_controller",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_library(
    name = "control_util",
    srcs = ["controlUtil.cpp"],
    hdrs = ["controlUtil.h"],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/math:autodiff",
        "//drake/math:expmap",
        "//drake/math:geometric_transform",
        "//drake/multibody:rigid_body_tree",
        "//drake/util",
    ],
)

drake_cc_library(
    name = "qp_common",
    srcs = [],
    hdrs = ["QPCommon.h"],
    deps = [
        ":control_util",
        "//drake/common",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/robotInterfaces:side",
        "//drake/util",
    ],
)

drake_cc_library(
    name = "zmp_planner",
    srcs = ["zmp_planner.cc"],
    hdrs = ["zmp_planner.h"],
    deps = [
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/systems/controllers:linear_quadratic_regulator",
    ],
)

drake_cc_library(
    name = "zmp_util",
    srcs = ["zmpUtil.cpp"],
    hdrs = ["zmpUtil.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "inverse_dynamics_test",
    data = [
        "//drake/examples/SimpleFourBar:models",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    deps = [
        ":inverse_dynamics",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "inverse_dynamics_controller_test",
    data = [
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    deps = [
        ":inverse_dynamics_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "linear_quadratic_regulator_test",
    deps = [
        ":linear_quadratic_regulator",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "pid_controlled_system_test",
    deps = [
        ":pid_controlled_system",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "pid_controller_test",
    deps = [
        ":pid_controller",
        "//drake/common:eigen_matrix_compare",
    ],
)

cpplint()
