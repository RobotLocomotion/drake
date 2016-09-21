if(lcm_FOUND)
  add_executable(lcm_subscriber_system_test lcm_subscriber_system_test.cc)
  target_link_libraries(lcm_subscriber_system_test
    drakeLCMSystem2
    ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME lcm_subscriber_system_test COMMAND lcm_subscriber_system_test)

  add_executable(lcm_publisher_system_test lcm_publisher_system_test.cc)
  target_link_libraries(lcm_publisher_system_test
    drakeLCMSystem2
    ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME lcm_publisher_system_test COMMAND lcm_publisher_system_test)

  add_executable(lcm_translator_dictionary_test
    lcm_translator_dictionary_test.cc)
  target_link_libraries(lcm_translator_dictionary_test
    drakeLCMSystem2
    ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME lcm_translator_dictionary_test COMMAND
    lcm_translator_dictionary_test)
endif()
