if (GTEST_FOUND AND LCM_FOUND AND NOT WIN32)
  add_executable(simple_car_test simple_car_test.cc)
  target_link_libraries(simple_car_test ${GTEST_BOTH_LIBRARIES} drakeCars)
  add_test(NAME simple_car_test COMMAND simple_car_test)
endif ()

if (GTEST_FOUND AND roscpp_FOUND)
  add_test(NAME car_sim_lcm_and_ros COMMAND car_sim_lcm_and_ros models/prius/prius.sdf models/stata_garage_p1.sdf --duration 5 WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/examples/Cars")
endif ()
