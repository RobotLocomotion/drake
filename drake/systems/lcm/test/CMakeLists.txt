if(lcm_FOUND)
  drake_add_cc_test(lcm_subscriber_system_test)
  target_link_libraries(lcm_subscriber_system_test
      drakeLcmSystem)

  drake_add_cc_test(lcm_publisher_system_test)
  target_link_libraries(lcm_publisher_system_test
      drakeLcmSystem
      drakeSystemAnalysis)

  drake_add_cc_test(lcm_translator_dictionary_test)
  target_link_libraries(lcm_translator_dictionary_test
      drakeLcmSystem)

  drake_add_cc_test(lcm_driven_loop_test)
  target_link_libraries(lcm_driven_loop_test
      drakeLcmDrivenLoop
      drakeSystemPrimitives)

  drake_add_cc_test(serializer_test)
  target_link_libraries(serializer_test drakeLcmSystem)

  drake_add_cc_test(translator_test)
  target_link_libraries(translator_test drakeLcmSystem)
endif()
