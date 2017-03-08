# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")

package(
    default_visibility = [":__subpackages__"],
)

drake_cc_library(
    name = "control_utils",
    srcs = [],
    hdrs = ["control_utils.h"],
    linkstatic = 1,
    deps = [
        "//drake/common",
        "//drake/math:geometric_transform",
    ],
)

exports_files(
    ["config/valkyrie.alias_groups"],
)

drake_cc_library(
    name = "humanoid_status",
    srcs = ["humanoid_status.cc"],
    hdrs = ["humanoid_status.h"],
    linkstatic = 1,
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
    linkstatic = 1,
    deps = [
        "//drake/multibody:rigid_body_tree",
    ],
)

drake_cc_library(
    name = "lcm_utils",
    srcs = ["lcm_utils.cc"],
    hdrs = ["lcm_utils.h"],
    deps = [
        "@bot_core_lcmtypes//:lib",
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
    ],
)

drake_cc_library(
    name = "qp_controller",
    srcs = ["qp_controller.cc"],
    hdrs = ["qp_controller.h"],
    linkstatic = 1,
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
    srcs = ["test/valkyrie_balancing_test.cc"],
    data = [
        "config/alias_groups.yaml",
        "config/controller.yaml",
        "//drake/examples/Valkyrie:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":control_utils",
        ":qp_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/Valkyrie:valkyrie_constants",
        "//drake/multibody/parsers",
    ],
)

cpplint()
