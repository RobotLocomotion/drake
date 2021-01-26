The follow lcmtypes are unused within Drake and therefore are being removed:

- lcmt_body_acceleration
- lcmt_body_motion_data
- lcmt_body_wrench_data
- lcmt_constrained_values
- lcmt_contact_information
- lcmt_desired_body_motion
- lcmt_desired_centroidal_momentum_dot
- lcmt_desired_dof_motions
- lcmt_inverse_dynamics_debug_info
- lcmt_joint_pd_override
- lcmt_manipulator_plan_move_end_effector
- lcmt_piecewise_polynomial
- lcmt_plan_eval_debug_info
- lcmt_polynomial
- lcmt_polynomial_matrix
- lcmt_qp_controller_input
- lcmt_qp_input
- lcmt_resolved_contact
- lcmt_support_data
- lcmt_viewer2_comms
- lcmt_whole_body_data
- lcmt_zmp_com_observer_state
- lcmt_zmp_data

The follow lcmtypes were only available in Python or Java (not C++), and are
also unused within Drake and therefore are being removed:

- lcmt_foot_flag
- lcmt_quadrotor_input_t
- lcmt_quadrotor_output_t
- lcmt_scope_data
- lcmt_simulation_command

The deprecated code will be removed from Drake on or after 2021-04-01.
