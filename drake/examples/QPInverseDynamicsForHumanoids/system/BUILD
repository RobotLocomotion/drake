# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

drake_cc_library(
    name = "humanoid_status_translator_system",
    srcs = ["humanoid_status_translator_system.cc"],
    hdrs = ["humanoid_status_translator_system.h"],
    deps = [
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids:lcm_utils",
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
        "@bot_core_lcmtypes//:lib",
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
        "//drake/common:drake_path",
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:humanoid_status",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:rigid_body_tree_alias_groups",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "humanoid_plan_eval_system",
    srcs = ["humanoid_plan_eval_system.cc"],
    hdrs = ["humanoid_plan_eval_system.h"],
    deps = [
        ":plan_eval_base_system",
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
        "//drake/systems/controllers:model_based_controller_base",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "valkyrie_balancing_controller_system",
    srcs = [
        "valkyrie_balancing_controller_system.cc",
    ],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.id_controller_config",
        "//drake/examples/Valkyrie:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":atlas_joint_level_controller_system",
        ":humanoid_plan_eval_system",
        ":humanoid_status_translator_system",
        ":qp_controller_system",
        "//drake/examples/Valkyrie:valkyrie_constants",
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_value_source",
        "@bot_core_lcmtypes//:lib",
    ],
)

drake_cc_googletest(
    name = "humanoid_plan_eval_test",
    srcs = ["test/humanoid_plan_eval_system_test.cc"],
    data = [
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/valkyrie.id_controller_config",
        "//drake/examples/Valkyrie:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":humanoid_plan_eval_system",
        ":qp_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids:qp_controller",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
        "//drake/examples/Valkyrie:valkyrie_constants",
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
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":manipulator_inverse_dynamics_controller",
        "//drake/common:eigen_matrix_compare",
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
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.alias_groups",
        "//drake/examples/QPInverseDynamicsForHumanoids:config/iiwa.id_controller_config",
        "//drake/examples/kuka_iiwa_arm:models",
    ],
    tags = [
        "exclusive",
        "gurobi",
        "local",
    ],
    deps = [
        ":qp_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/QPInverseDynamicsForHumanoids:control_utils",
        "//drake/examples/QPInverseDynamicsForHumanoids/param_parsers:param_parser",
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
        "//drake/examples/Valkyrie:models",
    ],
    deps = [
        ":atlas_joint_level_controller_system",
        "//drake/common:eigen_matrix_compare",
        "//drake/multibody/parsers",
        "//drake/systems/framework",
    ],
)

cpplint()
