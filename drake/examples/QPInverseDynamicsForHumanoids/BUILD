# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:gurobi.bzl", "gurobi_test_tags")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "control_utils",
    srcs = [],
    hdrs = ["control_utils.h"],
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

exports_files(
    ["config/valkyrie.alias_groups"],
)

exports_files(
    ["config/valkyrie.id_controller_config"],
)

exports_files(
    ["config/iiwa.alias_groups"],
)

exports_files(
    ["config/iiwa.id_controller_config"],
)

drake_cc_library(
    name = "humanoid_status",
    srcs = ["humanoid_status.cc"],
    hdrs = ["humanoid_status.h"],
    deps = [
        "//drake/common",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/robotInterfaces:side",
    ],
)

drake_cc_library(
    name = "qp_controller_common",
    srcs = ["qp_controller_common.cc"],
    hdrs = ["qp_controller_common.h"],
    deps = [
        "//drake/multibody:rigid_body_tree",
    ],
)

drake_cc_library(
    name = "lcm_utils",
    srcs = ["lcm_utils.cc"],
    hdrs = ["lcm_utils.h"],
    deps = [
        ":qp_controller_common",
        "//drake/lcmtypes:body_acceleration",
        "//drake/lcmtypes:constrained_values",
        "//drake/lcmtypes:contact_information",
        "//drake/lcmtypes:desired_body_motion",
        "//drake/lcmtypes:desired_centroidal_momentum_dot",
        "//drake/lcmtypes:desired_dof_motions",
        "//drake/lcmtypes:qp_input",
        "//drake/lcmtypes:resolved_contact",
        "//drake/util:lcm_util",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_library(
    name = "qp_controller",
    srcs = ["qp_controller.cc"],
    hdrs = ["qp_controller.h"],
    deps = [
        ":humanoid_status",
        ":qp_controller_common",
        "//drake/common",
        "//drake/math:gradient",
        "//drake/solvers:mathematical_program",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "lcm_utils_test",
    data = [
        "//drake/examples/Valkyrie:models",
    ],
    deps = [
        ":lcm_utils",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "setpoint_test",
    deps = [
        ":control_utils",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_googletest(
    name = "valkyrie_balancing_test",
    size = "medium",
    srcs = ["test/valkyrie_balancing_test.cc"],
    data = [
        "config/valkyrie.alias_groups",
        "config/valkyrie.id_controller_config",
        "//drake/examples/Valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":control_utils",
        ":qp_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/Valkyrie:valkyrie_constants",
        "//drake/multibody/parsers",
    ],
)

drake_cc_googletest(
    name = "iiwa_inverse_dynamics_test",
    srcs = ["test/iiwa_inverse_dynamics_test.cc"],
    data = [
        "config/iiwa.alias_groups",
        "config/iiwa.id_controller_config",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":control_utils",
        ":qp_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/multibody/parsers",
    ],
)

cpplint()
