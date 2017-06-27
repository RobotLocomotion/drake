# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)

package(
    default_visibility = [
        "//drake/examples/QPInverseDynamicsForHumanoids:__subpackages__",
        "//drake/examples/kuka_iiwa_arm:__subpackages__",
    ],
)

drake_cc_library(
    name = "generic_plan",
    srcs = [
        "generic_plan.cc",
    ],
    hdrs = [
        "generic_plan.h",
    ],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller_common",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/manipulation/util:trajectory_utils",
    ],
)

drake_cc_library(
    name = "manipulator_plans",
    srcs = [
        "manipulator_move_end_effector_plan.cc",
    ],
    hdrs = [
        "manipulator_move_end_effector_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/common:unused",
        "//drake/lcmtypes:manipulator_plan_move_end_effector",
        "//drake/util:lcm_util",
    ],
)

drake_cc_googletest(
    name = "manipulator_plan_test",
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":manipulator_plans",
        ":test_common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

# === test/ ===
drake_cc_googletest(
    name = "generic_plan_test",
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":generic_plan",
        ":test_common",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "test_common",
    hdrs = [
        "test/test_common.h",
    ],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
    ],
)

cpplint()
