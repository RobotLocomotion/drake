if(lcm_FOUND)
  add_executable(kuka_iiwa_arm_test kuka_iiwa_arm_test.cc)
  target_link_libraries(kuka_iiwa_arm_test
      drakeKukaIIWAArm
      ${GTEST_BOTH_LIBRARIES})
  add_test(NAME kuka_iiwa_arm_test COMMAND kuka_iiwa_arm_test)
endif()
