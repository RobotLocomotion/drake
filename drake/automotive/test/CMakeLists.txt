if(lcm_FOUND)
  add_executable(curve2_test curve2_test.cc)
  target_link_libraries(curve2_test ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  drake_add_test(NAME curve2_test COMMAND curve2_test)

  add_executable(simple_car_test simple_car_test.cc
    simple_car_scalartype_test.cc)
  target_link_libraries(simple_car_test ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  drake_add_test(NAME simple_car_test COMMAND simple_car_test)

  add_executable(simple_car_state_translator_test
    simple_car_state_translator_test.cc)
  target_link_libraries(simple_car_state_translator_test
    ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  add_test(NAME simple_car_state_translator_test
    COMMAND simple_car_state_translator_test)

  add_executable(idm_with_trajectory_agent_test
    idm_with_trajectory_agent_test.cc
    idm_with_trajectory_agent_scalartype_test.cc)
  target_link_libraries(idm_with_trajectory_agent_test
    ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  drake_add_test(NAME idm_with_trajectory_agent_test
    COMMAND idm_with_trajectory_agent_test)

  add_executable(simple_car_to_euler_floating_joint_test
    simple_car_to_euler_floating_joint_test.cc)
  target_link_libraries(simple_car_to_euler_floating_joint_test
    ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  add_test(NAME simple_car_to_euler_floating_joint_test
    COMMAND simple_car_to_euler_floating_joint_test)
  
  add_executable(trajectory_car_test trajectory_car_test.cc)
  target_link_libraries(trajectory_car_test
    ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  drake_add_test(NAME trajectory_car_test COMMAND trajectory_car_test)

  add_executable(automotive_simulator_test automotive_simulator_test.cc)
  target_link_libraries(automotive_simulator_test
    ${GTEST_BOTH_LIBRARIES} drakeAutomotive)
  drake_add_test(NAME automotive_simulator_test
    COMMAND automotive_simulator_test)
endif()
