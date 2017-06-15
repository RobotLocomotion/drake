# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "state_feedback_controller_interface",
    hdrs = ["state_feedback_controller_interface.h"],
    deps = [
        "//drake/systems/framework:input_port_descriptor",
        "//drake/systems/framework:system",
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
        ":pid_controller",
        ":state_feedback_controller_interface",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:pass_through",
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
        ":state_feedback_controller_interface",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/primitives:matrix_gain",
    ],
)

drake_cc_library(
    name = "pid_controlled_system",
    srcs = ["pid_controlled_system.cc"],
    hdrs = ["pid_controlled_system.h"],
    deps = [
        ":pid_controller",
        "//drake/systems/primitives:adder",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:saturation",
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

# This library is empty unless Gurobi is available.
drake_cc_library(
    name = "instantaneous_qp_controller",
    srcs = select({
        "//tools:with_gurobi": ["InstantaneousQPController.cpp"],
        "//conditions:default": [],
    }),
    hdrs = select({
        "//tools:with_gurobi": ["InstantaneousQPController.h"],
        "//conditions:default": [],
    }),
    deps = [
        ":control_util",
        ":qp_common",
        "//drake/common",
        "//drake/common:drake_path",
        "//drake/common:is_approx_equal_abstol",
        "//drake/lcmtypes:qp_controller_input",
        "//drake/lcmtypes:zmp_com_observer_state",
        "//drake/math:geometric_transform",
        "//drake/multibody/joints",
        "//drake/multibody/parsers",
        "//drake/solvers:gurobi_qp",
        "//drake/util",
        "//drake/util:lcm_util",
        "//drake/util:yaml_util",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "inverse_dynamics_test",
    data = [
        "//drake/examples/SimpleFourBar:models",
        "//drake/manipulation/models/iiwa_description:models",
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
        "//drake/manipulation/models/iiwa_description:models",
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

# The extra_srcs are required here because cpplint() doesn't understand how to
# extract labels from select() functions yet.
cpplint(extra_srcs = [
    "InstantaneousQPController.cpp",
    "InstantaneousQPController.h",
])
