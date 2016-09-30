if(lcm_FOUND)
  drake_add_cc_test(drake_lcm_test)
  target_link_libraries(drake_lcm_test
      drakeLCMTypes
      drakeLcm)
endif()
