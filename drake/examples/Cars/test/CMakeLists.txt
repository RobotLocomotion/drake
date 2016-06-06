if(GTEST_FOUND AND LCM_FOUND AND NOT WIN32)
  add_executable(curve2_test curve2_test.cc)
  target_link_libraries(curve2_test ${GTEST_BOTH_LIBRARIES} drakeCars)
  add_test(NAME curve2_test COMMAND curve2_test)

  add_executable(simple_car_test simple_car_test.cc)
  target_link_libraries(simple_car_test ${GTEST_BOTH_LIBRARIES} drakeCars)
  add_test(NAME simple_car_test COMMAND simple_car_test)

  add_executable(trajectory_car_test trajectory_car_test.cc)
  target_link_libraries(trajectory_car_test ${GTEST_BOTH_LIBRARIES} drakeCars)
  add_test(NAME trajectory_car_test COMMAND trajectory_car_test)
endif()
