if(lcm_FOUND)
  add_executable(kuka_iiwa_dynamics_test kuka_iiwa_dynamics_test.cc)
  target_link_libraries(kuka_iiwa_dynamics_test
      drakeKukaIiwaArm
      ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME kuka_iiwa_dynamics_test COMMAND kuka_iiwa_dynamics_test)

  add_executable(kuka_iiwa_pd_control_test
          kuka_iiwa_pd_control_test.cc)
  target_link_libraries(kuka_iiwa_pd_control_test
          drakeKukaIiwaArm
          ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME kuka_iiwa_pd_control_test
          COMMAND kuka_iiwa_pd_control_test)

  add_executable(kuka_iiwa_gravity_compensated_position_control_test
          kuka_iiwa_gravity_compensated_position_control_test.cc)
  target_link_libraries(kuka_iiwa_gravity_compensated_position_control_test
          drakeKukaIiwaArm
          ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME kuka_iiwa_gravity_compensated_position_control_test
          COMMAND kuka_iiwa_gravity_compensated_position_control_test
          SIZE medium)

  add_executable(kuka_iiwa_gravity_compensated_torque_control_test
          kuka_iiwa_gravity_compensated_torque_control_test.cc)
  target_link_libraries(kuka_iiwa_gravity_compensated_torque_control_test
          drakeKukaIiwaArm
          ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME kuka_iiwa_gravity_compensated_torque_control_test
          COMMAND kuka_iiwa_gravity_compensated_torque_control_test
          SIZE medium)
endif()
