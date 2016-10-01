if(lcm_FOUND)
  drake_add_cc_test(drake_lcm_test)
  target_link_libraries(drake_lcm_test
      drakeLCMTypes
      drakeLcm)
endif()

drake_add_cc_test(drake_mock_lcm_test)
target_link_libraries(drake_mock_lcm_test
  drakeLcm)