# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
    "drake_cc_googletest",
)

package(
    default_visibility = [":__subpackages__"],
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "iiwa_common",
    srcs = [
        "iiwa_common.cc",
    ],
    hdrs = [
        "iiwa_common.h",
    ],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "@bot_core_lcmtypes",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_library(
    name = "iiwa_lcm",
    srcs = ["iiwa_lcm.cc"],
    hdrs = ["iiwa_lcm.h"],
    deps = [
        ":iiwa_common",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "oracular_state_estimator",
    srcs = ["oracular_state_estimator.cc"],
    hdrs = ["oracular_state_estimator.h"],
    deps = [
        "//drake/manipulation/util:robot_state_msg_translator",
        "//drake/systems/framework:leaf_system",
        "//drake/util:lcm_util",
        "@bot_core_lcmtypes",
    ],
)

drake_cc_library(
    name = "robot_plan_interpolator",
    srcs = ["robot_plan_interpolator.cc"],
    hdrs = ["robot_plan_interpolator.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/systems/framework:leaf_system",
        "@bot_core_lcmtypes",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_library(
    name = "sim_diagram_builder",
    srcs = ["sim_diagram_builder.cc"],
    hdrs = ["sim_diagram_builder.h"],
    deps = [
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/controllers:state_feedback_controller_interface",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "iiwa_controller",
    srcs = ["iiwa_controller.cc"],
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        ":robot_plan_interpolator",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/lcm",
        "//drake/systems/lcm:lcm_driven_loop",
        "//drake/systems/primitives:demultiplexer",
        "@bot_core_lcmtypes",
        "@com_github_gflags_gflags//:gflags",
        "@robotlocomotion_lcmtypes",
    ],
)

drake_cc_binary(
    name = "iiwa_wsg_simulation",
    srcs = ["iiwa_wsg_simulation.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/wsg_50_description:models",
    ],
    test_rule_args = ["--simulation_sec=0.01"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        ":oracular_state_estimator",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:iiwa_wsg_diagram_factory",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/manipulation/schunk_wsg:schunk_wsg_constants",
        "//drake/manipulation/schunk_wsg:schunk_wsg_lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:matrix_gain",
        "//drake/util:lcm_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "kuka_simulation",
    srcs = ["kuka_simulation.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        ":sim_diagram_builder",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:constant_vector_source",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "kuka_plan_runner",
    srcs = ["kuka_plan_runner.cc"],
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_common",
        ":iiwa_lcm",
        "@bot_core_lcmtypes",
        "@robotlocomotion_lcmtypes",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "iiwa_lcm_test",
    deps = [
        ":iiwa_lcm",
        "//drake/common:eigen_matrix_compare",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "robot_plan_interpolator_test",
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_common",
        ":robot_plan_interpolator",
        "//drake/systems/framework",
    ],
)

drake_cc_googletest(
    name = "sim_diagram_builder_test",
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
    ],
    deps = [
        ":iiwa_common",
        ":sim_diagram_builder",
        "//drake/common:eigen_matrix_compare",
        "//drake/examples/kuka_iiwa_arm/iiwa_world:world_sim_tree_builder",
        "//drake/lcm",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/controllers:pid_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_googletest(
    name = "optitrack_test",
    deps = [
        "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
    ],
)

cpplint()
