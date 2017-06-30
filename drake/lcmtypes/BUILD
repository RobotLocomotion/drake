# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("//tools:lcm.bzl", "lcm_libraries", "lcm_py_library")

lcm_libraries(
    name = "acrobot",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_acrobot_u.lcm",
        "lcmt_acrobot_x.lcm",
        "lcmt_acrobot_y.lcm",
    ],
)

lcm_libraries(
    name = "body_acceleration",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_acceleration.lcm"],
)

lcm_libraries(
    name = "body_motion_data",
    cc_deps = [":piecewise_polynomial"],
    java_deps = [":piecewise_polynomial_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_motion_data.lcm"],
)

lcm_libraries(
    name = "body_wrench_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_body_wrench_data.lcm"],
)

lcm_libraries(
    name = "constrained_values",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_constrained_values.lcm"],
)

lcm_libraries(
    name = "contact_info_for_viz",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_info_for_viz.lcm"],
)

lcm_libraries(
    name = "contact_results_for_viz",
    cc_deps = [":contact_info_for_viz"],
    java_deps = [":contact_info_for_viz_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_results_for_viz.lcm"],
)

lcm_libraries(
    name = "contact_information",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_contact_information.lcm"],
)

lcm_libraries(
    name = "desired_body_motion",
    cc_deps = [":constrained_values"],
    java_deps = [":constrained_values_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_body_motion.lcm"],
)

lcm_libraries(
    name = "desired_centroidal_momentum_dot",
    cc_deps = [":constrained_values"],
    java_deps = [":constrained_values_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_centroidal_momentum_dot.lcm"],
)

lcm_libraries(
    name = "desired_dof_motions",
    cc_deps = [":constrained_values"],
    java_deps = [":constrained_values_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_desired_dof_motions.lcm"],
)

lcm_libraries(
    name = "inverse_dynamics_debug_info",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_inverse_dynamics_debug_info.lcm"],
)

lcm_libraries(
    name = "joint_pd_override",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_joint_pd_override.lcm"],
)

lcm_libraries(
    name = "manipulator_plan_move_end_effector",
    cc_deps = ["@lcmtypes_bot2_core"],
    java_deps = ["@lcmtypes_bot2_core//:lcmtypes_bot2_core_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_manipulator_plan_move_end_effector.lcm"],
)

lcm_libraries(
    name = "piecewise_polynomial",
    cc_deps = [":polynomial_matrix"],
    java_deps = [":polynomial_matrix_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_piecewise_polynomial.lcm"],
)

lcm_libraries(
    name = "polynomial",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_polynomial.lcm"],
)

lcm_libraries(
    name = "plan_eval_debug_info",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_plan_eval_debug_info.lcm"],
)

lcm_libraries(
    name = "polynomial_matrix",
    cc_deps = [":polynomial"],
    java_deps = [":polynomial_java"],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_polynomial_matrix.lcm"],
)

lcm_libraries(
    name = "qp_controller_input",
    cc_deps = [
        ":body_motion_data",
        ":body_wrench_data",
        ":joint_pd_override",
        ":support_data",
        ":whole_body_data",
        ":zmp_data",
    ],
    java_deps = [
        ":body_motion_data_java",
        ":body_wrench_data_java",
        ":joint_pd_override_java",
        ":support_data_java",
        ":whole_body_data_java",
        ":zmp_data_java",
    ],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_qp_controller_input.lcm"],
)

lcm_libraries(
    name = "qp_input",
    cc_deps = [
        ":contact_information",
        ":desired_body_motion",
        ":desired_centroidal_momentum_dot",
        ":desired_dof_motions",
    ],
    java_deps = [
        ":contact_information_java",
        ":desired_dof_motions_java",
        ":desired_centroidal_momentum_dot_java",
        ":desired_body_motion_java",
        ":desired_dof_motions_java",
    ],
    lcm_package = "drake",
    lcm_srcs = ["lcmt_qp_input.lcm"],
)

lcm_libraries(
    name = "resolved_contact",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_resolved_contact.lcm"],
)

lcm_libraries(
    name = "support_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_support_data.lcm"],
)

lcm_libraries(
    name = "whole_body_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_whole_body_data.lcm"],
)

lcm_libraries(
    name = "zmp_data",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_zmp_data.lcm"],
)

lcm_libraries(
    name = "zmp_com_observer_state",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_zmp_com_observer_state.lcm"],
)

lcm_libraries(
    name = "drake_signal",
    lcm_package = "drake",
    lcm_srcs = ["lcmt_drake_signal.lcm"],
)

lcm_libraries(
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

lcm_libraries(
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

lcm_libraries(
    name = "iiwa",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_iiwa_command.lcm",
        "lcmt_iiwa_status.lcm",
    ],
)

lcm_libraries(
    name = "schunk",
    lcm_package = "drake",
    lcm_srcs = [
        "lcmt_schunk_wsg_command.lcm",
        "lcmt_schunk_wsg_status.lcm",
    ],
)

# TODO(jwnimmer-tri) Many more C++ messages are missing ...

lcm_py_library(
    name = "lcmtypes-py",
    lcm_package = "drake",
    lcm_srcs = glob(["*.lcm"]),
    deps = [
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
    ],
)

# This rule should list every LCM type that is known to Drake or its external
# dependencies.
java_binary(
    name = "drake-lcm-spy",
    main_class = "lcm.spy.Spy",
    runtime_deps = [
        ":acrobot_java",
        ":automotive_java",
        ":body_acceleration_java",
        ":body_motion_data_java",
        ":body_wrench_data_java",
        ":constrained_values_java",
        ":contact_info_for_viz_java",
        ":contact_information_java",
        ":contact_results_for_viz_java",
        ":desired_body_motion_java",
        ":desired_centroidal_momentum_dot_java",
        ":desired_dof_motions_java",
        ":drake_signal_java",
        ":iiwa_java",
        ":inverse_dynamics_debug_info_java",
        ":joint_pd_override_java",
        ":manipulator_plan_move_end_effector_java",
        ":piecewise_polynomial_java",
        ":plan_eval_debug_info_java",
        ":polynomial_java",
        ":polynomial_matrix_java",
        ":qp_controller_input_java",
        ":qp_input_java",
        ":resolved_contact_java",
        ":schunk_java",
        ":support_data_java",
        ":viewer_java",
        ":whole_body_data_java",
        ":zmp_com_observer_state_java",
        ":zmp_data_java",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_java",
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_java",
        "@libbot//:lcmtypes_bot2_frames_java",
        "@libbot//:lcmtypes_bot2_lcmgl_java",
        "@libbot//:lcmtypes_bot2_param_java",
    ],
)

# === test/ ===

py_test(
    name = "polynomial_matrix_test",
    srcs = ["test/polynomial_matrix_test.py"],
    deps = [
        ":lcmtypes-py",
    ],
)
