add_executable(spring_mass_system_test spring_mass_system_test.cc)
target_link_libraries(spring_mass_system_test drakeSpringMassSystem drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME spring_mass_system_test COMMAND spring_mass_system_test)

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
