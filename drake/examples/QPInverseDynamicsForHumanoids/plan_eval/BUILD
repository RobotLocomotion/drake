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
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",  # noqa
        "//drake/manipulation/util:trajectory_utils",
        "//drake/systems/framework:value",
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
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
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

drake_cc_library(
    name = "humanoid_manipulation_plan",
    srcs = [
        "dev/humanoid_manipulation_plan.cc",
    ],
    hdrs = [
        "dev/humanoid_manipulation_plan.h",
    ],
    deps = [
        ":generic_plan",
        "//drake/examples/QPInverseDynamicsForHumanoids:lcm_utils",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/systems/controllers:zmp_planner",
        "//drake/util",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

drake_cc_binary(
    name = "humanoid_manipulation_demo",
    srcs = [
        "dev/humanoid_manipulation_demo.cc",
    ],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@lcmtypes_bot2_core",
        "@lcmtypes_robotlocomotion",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "generic_plan_test",
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
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
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",  # noqa
    ],
)

cpplint()
