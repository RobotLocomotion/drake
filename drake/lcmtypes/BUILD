# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("@drake//tools:install.bzl", "install")
load("//tools:lcm.bzl", "lcm_cc_library", "lcm_py_library", "lcm_java_library")
load("//tools:lint.bzl", "add_lint_tests")

lcm_cc_library(
    name = "acrobot",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_acrobot_u.lcm",
        "lcmt_acrobot_x.lcm",
        "lcmt_acrobot_y.lcm",
    ],
)

lcm_cc_library(
    name = "body_acceleration",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_acceleration.lcm"],
)

lcm_cc_library(
    name = "body_motion_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_motion_data.lcm"],
    deps = [":piecewise_polynomial"],
)

lcm_cc_library(
    name = "body_wrench_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_wrench_data.lcm"],
)

lcm_cc_library(
    name = "constrained_values",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_constrained_values.lcm"],
)

lcm_cc_library(
    name = "contact_info_for_viz",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_info_for_viz.lcm"],
)

lcm_cc_library(
    name = "contact_results_for_viz",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_results_for_viz.lcm"],
    deps = [":contact_info_for_viz"],
)

lcm_cc_library(
    name = "contact_information",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_information.lcm"],
)

lcm_cc_library(
    name = "desired_body_motion",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_body_motion.lcm"],
    deps = [":constrained_values"],
)

lcm_cc_library(
    name = "desired_centroidal_momentum_dot",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_centroidal_momentum_dot.lcm"],
    deps = [":constrained_values"],
)

lcm_cc_library(
    name = "desired_dof_motions",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_dof_motions.lcm"],
    deps = [":constrained_values"],
)

lcm_cc_library(
    name = "inverse_dynamics_debug_info",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_inverse_dynamics_debug_info.lcm"],
)

lcm_cc_library(
    name = "joint_pd_override",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_joint_pd_override.lcm"],
)

lcm_cc_library(
    name = "joint_positions",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_joint_positions.lcm"],
)


lcm_cc_library(
    name = "manipulator_plan_move_end_effector",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_manipulator_plan_move_end_effector.lcm"],
    deps = ["@lcmtypes_bot2_core"],
)

lcm_cc_library(
    name = "piecewise_polynomial",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_piecewise_polynomial.lcm"],
    deps = [":polynomial_matrix"],
)

lcm_cc_library(
    name = "polynomial",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_polynomial.lcm"],
)

lcm_cc_library(
    name = "plan_eval_debug_info",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_plan_eval_debug_info.lcm"],
)

lcm_cc_library(
    name = "polynomial_matrix",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_polynomial_matrix.lcm"],
    deps = [":polynomial"],
)

lcm_cc_library(
    name = "qp_controller_input",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_qp_controller_input.lcm"],
    deps = [
        ":body_motion_data",
        ":body_wrench_data",
        ":joint_pd_override",
        ":support_data",
        ":whole_body_data",
        ":zmp_data",
    ],
)

lcm_cc_library(
    name = "qp_input",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_qp_input.lcm"],
    deps = [
        ":contact_information",
        ":desired_body_motion",
        ":desired_centroidal_momentum_dot",
        ":desired_dof_motions",
    ],
)

lcm_cc_library(
    name = "resolved_contact",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_resolved_contact.lcm"],
)

lcm_cc_library(
    name = "robot_state",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_robot_state.lcm"],
)

lcm_cc_library(
    name = "support_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_support_data.lcm"],
)

lcm_cc_library(
    name = "whole_body_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_whole_body_data.lcm"],
)

lcm_cc_library(
    name = "zmp_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_zmp_data.lcm"],
)

lcm_cc_library(
    name = "zmp_com_observer_state",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_zmp_com_observer_state.lcm"],
)

lcm_cc_library(
    name = "drake_signal",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
)

lcm_cc_library(
    name = "viewer",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_viewer_command.lcm",
        "lcmt_viewer_draw.lcm",
        "lcmt_viewer_geometry_data.lcm",
        "lcmt_viewer_link_data.lcm",
        "lcmt_viewer_load_robot.lcm",
    ],
)

lcm_cc_library(
    name = "automotive",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_bicycle_car_parameters_t.lcm",
        "lcmt_bicycle_car_state_t.lcm",
        "lcmt_driving_command_t.lcm",
        "lcmt_endless_road_car_config_t.lcm",
        "lcmt_endless_road_car_state_t.lcm",
        "lcmt_endless_road_oracle_output_t.lcm",
        "lcmt_euler_floating_joint_state_t.lcm",
        "lcmt_maliput_railcar_params_t.lcm",
        "lcmt_maliput_railcar_state_t.lcm",
        "lcmt_simple_car_params_t.lcm",
        "lcmt_simple_car_state_t.lcm",
        "lcmt_trajectory_car_params_t.lcm",
        "lcmt_trajectory_car_state_t.lcm",
    ],
)

lcm_cc_library(
    name = "iiwa",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_iiwa_command.lcm",
        "lcmt_iiwa_status.lcm",
    ],
)

lcm_cc_library(
    name = "schunk",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_schunk_wsg_command.lcm",
        "lcmt_schunk_wsg_status.lcm",
    ],
)

# TODO(jwnimmer-tri) Many more C++ messages are missing ...

lcm_py_library(
    name = "lcmtypes_py",
    lcm_package = "drake",
    lcm_srcs = glob(["*.lcm"]),
    deps = [
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
    ],
)

lcm_java_library(
    name = "lcmtypes_drake_java",
    lcm_package = "drake",
    lcm_srcs = glob(["*.lcm"]),
    deps = ["@lcmtypes_bot2_core//:lcmtypes_bot2_core_java"],
)

# This should list every LCM type that is known to Drake.
DRAKE_LCMTYPES = [
    ":acrobot",
    ":automotive",
    ":body_acceleration",
    ":body_motion_data",
    ":body_wrench_data",
    ":constrained_values",
    ":contact_info_for_viz",
    ":contact_information",
    ":contact_results_for_viz",
    ":desired_body_motion",
    ":desired_centroidal_momentum_dot",
    ":desired_dof_motions",
    ":drake_signal",
    ":iiwa",
    ":inverse_dynamics_debug_info",
    ":joint_pd_override",
    ":manipulator_plan_move_end_effector",
    ":piecewise_polynomial",
    ":plan_eval_debug_info",
    ":polynomial",
    ":polynomial_matrix",
    ":qp_controller_input",
    ":qp_input",
    ":resolved_contact",
    ":schunk",
    ":support_data",
    ":viewer",
    ":whole_body_data",
    ":zmp_data",
    ":zmp_com_observer_state",
    ":robot_state",
    ":joint_positions",
]

# This rule should list every LCM type that is known to Drake or its external
# dependencies.
java_binary(
    name = "drake-lcm-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [
        ":lcmtypes_drake_java",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_java",
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_java",
        "@libbot//:lcmtypes_bot2_frames_java",
        "@libbot//:lcmtypes_bot2_lcmgl_java",
        "@libbot//:lcmtypes_bot2_param_java",
    ],
)

install(
    name = "install",
    targets = DRAKE_LCMTYPES + [
        ":lcmtypes_drake_java",
        ":lcmtypes_py",
    ],
    hdr_dest = "include/drake/lcmtypes",
    guess_hdrs = "PACKAGE",
    rename = {"share/java/liblcmtypes_drake_java.jar": "lcmtypes_drake.jar"},
)

# === test/ ===

py_test(
    name = "polynomial_matrix_test",
    srcs = ["test/polynomial_matrix_test.py"],
    deps = [
        ":lcmtypes_py",
    ],
)

add_lint_tests()
