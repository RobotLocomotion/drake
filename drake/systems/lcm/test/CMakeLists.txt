if(lcm_FOUND)
  add_executable(lcm_subscriber_system_test lcm_subscriber_system_test.cc)
  target_link_libraries(lcm_subscriber_system_test
    drakeLCMSystem2
    ${GTEST_BOTH_LIBRARIES})
  add_test(NAME lcm_subscriber_system_test COMMAND lcm_subscriber_system_test)
endif()
