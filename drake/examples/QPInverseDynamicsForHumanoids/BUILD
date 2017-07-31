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
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_alias_groups",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state",
        "//drake/systems/robotInterfaces:side",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "valkyrie_balancing_test",
    size = "medium",
    srcs = ["test/valkyrie_balancing_test.cc"],
    data = [
        "config/valkyrie.alias_groups",
        "config/valkyrie.id_controller_config",
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":humanoid_status",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/multibody/parsers",
        "//drake/systems/controllers:setpoint",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",  # noqa
        "//drake/systems/controllers/qp_inverse_dynamics:qp_inverse_dynamics",
    ],
)

cpplint()
