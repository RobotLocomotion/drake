# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("//tools:lcm.bzl", "lcm_cc_library", "lcm_java_library", "lcm_py_library")

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
    deps = [
        ":contact_info_for_viz",
    ],
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
)

lcm_cc_library(
    name = "desired_centroidal_momentum_dot",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_centroidal_momentum_dot.lcm"],
)

lcm_cc_library(
    name = "desired_dof_motions",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_dof_motions.lcm"],
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
    name = "piecewise_polynomial",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_piecewise_polynomial.lcm"],
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
)

lcm_cc_library(
    name = "resolved_contact",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_resolved_contact.lcm"],
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
    name = "drake_signal",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
)

_VIEWER_LCM_SRCS = [
    "lcmt_viewer_command.lcm",
    "lcmt_viewer_draw.lcm",
    "lcmt_viewer_geometry_data.lcm",
    "lcmt_viewer_link_data.lcm",
    "lcmt_viewer_load_robot.lcm",
]

lcm_cc_library(
    name = "viewer",
    lcm_package = "drake",
    lcm_srcs = _VIEWER_LCM_SRCS,
)

lcm_java_library(
    name = "viewer-java",
    lcm_package = "drake",
    lcm_srcs = _VIEWER_LCM_SRCS,
)

_AUTOMOTIVE_LCM_SRCS = [
    "lcmt_bicycle_car_parameters_t.lcm",
    "lcmt_bicycle_car_state_t.lcm",
    "lcmt_driving_command_t.lcm",
    "lcmt_endless_road_car_config_t.lcm",
    "lcmt_endless_road_car_state_t.lcm",
    "lcmt_endless_road_oracle_output_t.lcm",
    "lcmt_euler_floating_joint_state_t.lcm",
    "lcmt_maliput_railcar_config_t.lcm",
    "lcmt_maliput_railcar_state_t.lcm",
    "lcmt_simple_car_config_t.lcm",
    "lcmt_simple_car_state_t.lcm",
]

lcm_cc_library(
    name = "automotive",
    lcm_package = "drake",
    lcm_srcs = _AUTOMOTIVE_LCM_SRCS,
)

lcm_java_library(
    name = "automotive-java",
    lcm_package = "drake",
    lcm_srcs = _AUTOMOTIVE_LCM_SRCS,
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

lcm_cc_library(
    name = "call_matlab",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_call_matlab.lcm",
        "lcmt_matlab_array.lcm",
    ],
)

# TODO(jwnimmer-tri) Many more C++ messages are missing ...

lcm_py_library(
    name = "lcmtypes-py",
    lcm_package = "drake",
    lcm_srcs = glob(["*.lcm"]),
)

# === test/ ===

py_test(
    name = "polynomial_matrix_test",
    srcs = ["test/polynomial_matrix_test.py"],
    deps = [
        ":lcmtypes-py",
    ],
)
