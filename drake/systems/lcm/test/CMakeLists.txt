if(lcm_FOUND)
  add_executable(lcm_subscriber_system_test lcm_subscriber_system_test.cc)
  target_link_libraries(lcm_subscriber_system_test
    lcm
    drakeLCMSystem2
    drakeSpringMassSystem
    drakeSystemFramework
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT})
  add_test(NAME lcm_subscriber_system_test COMMAND lcm_subscriber_system_test)
endif()
