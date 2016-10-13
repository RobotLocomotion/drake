if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_system_test)
  target_link_libraries(simulated_schunk_system_test
    drakeSchunkGripper drakeSystemAnalysis)
endif()
