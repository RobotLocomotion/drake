if(lcm_FOUND)
  drake_add_cc_test(kuka_iiwa_dynamics_test)
  target_link_libraries(kuka_iiwa_dynamics_test drakeKukaIiwaArm)

  drake_add_cc_test(kuka_iiwa_pd_control_test)
  target_link_libraries(kuka_iiwa_pd_control_test drakeKukaIiwaArm)

  drake_add_cc_test(NAME kuka_iiwa_gravity_compensated_position_control_test
    SIZE medium)
  target_link_libraries(kuka_iiwa_gravity_compensated_position_control_test
    drakeKukaIiwaArm)

  drake_add_cc_test(NAME kuka_iiwa_gravity_compensated_torque_control_test
    SIZE medium)
  target_link_libraries(kuka_iiwa_gravity_compensated_torque_control_test
    drakeKukaIiwaArm)
endif()
