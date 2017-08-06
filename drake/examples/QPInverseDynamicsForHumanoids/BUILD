# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:drake.bzl", "drake_cc_googletest", "drake_cc_library")
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools:lint.bzl", "add_lint_tests")

package(
    default_visibility = [":__subpackages__"],
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
        "//drake/multibody:rigid_body_tree_alias_groups",
        "//drake/systems/controllers/qp_inverse_dynamics:robot_kinematic_state",  # noqa
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
        "//drake/systems/controllers/qp_inverse_dynamics",
        "//drake/systems/controllers/qp_inverse_dynamics:param_parser",
    ],
)

add_lint_tests()
