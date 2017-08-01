# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library",
)
load("//tools:gurobi.bzl", "gurobi_test_tags")
load("//tools:lint.bzl", "add_lint_tests")

drake_cc_library(
    name = "humanoid_status_translator_system",
    srcs = ["humanoid_status_translator_system.cc"],
    hdrs = ["humanoid_status_translator_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids:lcm_utils",
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "joint_level_controller_system",
    srcs = ["joint_level_controller_system.cc"],
    hdrs = ["joint_level_controller_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_library(
    name = "atlas_joint_level_controller_system",
    srcs = ["atlas_joint_level_controller_system.cc"],
    hdrs = ["atlas_joint_level_controller_system.h"],
    deps = [
        ":joint_level_controller_system",
    ],
)

drake_cc_library(
    name = "plan_eval_base_system",
    srcs = ["plan_eval_base_system.cc"],
    hdrs = ["plan_eval_base_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",  # noqa
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "humanoid_plan_eval_system",
    srcs = ["humanoid_plan_eval_system.cc"],
    hdrs = ["humanoid_plan_eval_system.h"],
    deps = [
        ":plan_eval_base_system",
        "//drake/examples/QPInverseDynamicsForHumanoids/plan_eval:humanoid_manipulation_plan",  # noqa
    ],
)

drake_cc_library(
    name = "manipulator_plan_eval_system",
    srcs = ["manipulator_plan_eval_system.cc"],
    hdrs = ["manipulator_plan_eval_system.h"],
    deps = [
        ":plan_eval_base_system",
        "//drake/lcmtypes:plan_eval_debug_info",
    ],
)

drake_cc_library(
    name = "qp_controller_system",
    srcs = ["qp_controller_system.cc"],
    hdrs = ["qp_controller_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/lcmtypes:inverse_dynamics_debug_info",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "manipulator_inverse_dynamics_controller",
    srcs = ["manipulator_inverse_dynamics_controller.cc"],
    hdrs = ["manipulator_inverse_dynamics_controller.h"],
    deps = [
        ":humanoid_status_translator_system",
        ":joint_level_controller_system",
        ":manipulator_plan_eval_system",
        ":qp_controller_system",
        "//drake/multibody/parsers",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/framework",
    ],
)

drake_cc_library(
    name = "valkyrie_controller",
    hdrs = ["valkyrie_controller.h"],
    deps = [
        ":atlas_joint_level_controller_system",
        ":humanoid_plan_eval_system",
        ":humanoid_status_translator_system",
        ":qp_controller_system",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "@lcmtypes_bot2_core",
    ],
)

drake_cc_binary(
    name = "valkyrie_balancing_controller_system",
    srcs = [
        "valkyrie_balancing_controller_system.cc",
    ],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.id_controller_config",  # noqa
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":valkyrie_controller",
        "//drake/common:find_resource",
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/systems/lcm:lcm_driven_loop",
    ],
)

drake_cc_googletest(
    name = "humanoid_plan_eval_test",
    srcs = ["test/humanoid_plan_eval_system_test.cc"],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.id_controller_config",  # noqa
        "//drake/examples/valkyrie:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":humanoid_plan_eval_system",
        ":qp_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",  # noqa
        "//drake/examples/valkyrie:valkyrie_constants",
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_value_source",
    ],
)

drake_cc_googletest(
    name = "manipulator_inverse_dynamics_controller_test",
    srcs = ["test/manipulator_inverse_dynamics_controller_test.cc"],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":manipulator_inverse_dynamics_controller",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_googletest(
    name = "qp_controller_system_test",
    srcs = ["test/qp_controller_system_test.cc"],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",  # noqa
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",  # noqa
        "//drake/manipulation/models/iiwa_description:models",
    ],
    tags = gurobi_test_tags(),
    deps = [
        ":qp_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",  # noqa
        "//drake/multibody/parsers",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_value_source",
    ],
)

drake_cc_googletest(
    name = "joint_level_controller_system_test",
    srcs = ["test/joint_level_controller_system_test.cc"],
    data = [
        "//drake/examples/valkyrie:models",
    ],
    deps = [
        ":atlas_joint_level_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/common:find_resource",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
    ],
)

add_lint_tests()
