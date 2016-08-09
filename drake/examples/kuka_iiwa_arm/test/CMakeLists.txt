if(lcm_FOUND)
  add_executable(kuka_iiwa_arm_test kuka_iiwa_arm_test.cc)
  target_link_libraries(kuka_iiwa_arm_test
      drakeKukaIiwaArm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME kuka_iiwa_arm_test COMMAND kuka_iiwa_arm_test)

  add_executable(kuka_iiwa_pd_control_test
          kuka_iiwa_pd_control_test.cc)
  target_link_libraries(kuka_iiwa_pd_control_test
          drakeKukaIiwaArm
          ${GTEST_BOTH_LIBRARIES})
  add_test(NAME kuka_iiwa_pd_control_test
          COMMAND kuka_iiwa_pd_control_test)

  add_executable(kuka_iiwa_gravity_compensation_test
          kuka_iiwa_gravity_compensation_test.cc)
  target_link_libraries(kuka_iiwa_gravity_compensation_test
          drakeKukaIiwaArm
          ${GTEST_BOTH_LIBRARIES})
  add_test(NAME kuka_iiwa_gravity_compensation_test
          COMMAND kuka_iiwa_gravity_compensation_test)
endif()
