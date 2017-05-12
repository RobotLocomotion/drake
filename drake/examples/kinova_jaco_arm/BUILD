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
        "@bot_core_lcmtypes//:lib",
        "@robotlocomotion_lcmtypes//:lib",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
    ],
)

drake_cc_library(
    name = "iiwa_lcm",
    srcs = ["iiwa_lcm.cc"],
    hdrs = ["iiwa_lcm.h"],
    deps = [
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcmtypes:iiwa",
        "//drake/multibody:inverse_kinematics",
        "//drake/multibody:rigid_body_tree",
        "//drake/systems/framework:leaf_system",
    ],
)

drake_cc_library(
    name = "sim_diagram_builder",
    srcs = ["sim_diagram_builder.cc"],
    hdrs = ["sim_diagram_builder.h"],
    deps = [
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/controllers:state_feedback_controller_base",
        "//drake/systems/framework",
    ],
)

drake_cc_binary(
    name = "run_kinova_jaco_arm_dynamics",
    srcs = ["run_kinova_jaco_arm_dynamics.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/jaco_description:models",
    ],
    test_rule_args = ["--duration=0.5"],
    test_rule_size = "medium",
    deps = [
        "@gflags//:gflags",
        ":iiwa_common",
        ":sim_diagram_builder",
        "//drake/common:drake_path",
        "//drake/common:text_logging_gflags",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/framework:context",
        "//drake/systems/framework:diagram",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_binary(
    name = "setpoint_jaco_demo",
    srcs = ["setpoint_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/jaco_description:models",
    ],
    tags = ["snopt"],
    deps = [
        "@gflags//:gflags",
        ":iiwa_common",
        ":sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:constant_vector_source",
    ],
)

drake_cc_binary(
    name = "controlled_jaco_demo",
    srcs = ["controlled_jaco_demo.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/manipulation/models/iiwa_description:models",
        "//drake/manipulation/models/jaco_description:models",
    ],
    tags = ["snopt"],
    deps = [
        "@gflags//:gflags",
        ":iiwa_common",
        ":sim_diagram_builder",
        "//drake/lcm",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/primitives:trajectory_source",
    ],
)

drake_cc_binary(
    name = "sean_test",
    srcs = ["sean-test.cc"],
    deps = ["//drake/common:drake_path"],
)

cpplint()
