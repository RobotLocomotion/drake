if(lcm_FOUND)
  drake_add_cc_test(lcm_subscriber_system_test)
  target_link_libraries(lcm_subscriber_system_test
      drakeLCMSystem2)

  drake_add_cc_test(lcm_publisher_system_test)
  target_link_libraries(lcm_publisher_system_test
      drakeLCMSystem2)

  drake_add_cc_test(NAME lcm_translator_dictionary_test)
  target_link_libraries(lcm_translator_dictionary_test
      drakeLCMSystem2)

  drake_add_cc_test(serializer_test)
  target_link_libraries(serializer_test drakeLCMSystem2)
endif()
