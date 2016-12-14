if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_wsg_system_test)
  target_link_libraries(simulated_schunk_wsg_system_test
    drakeSchunkWsg drakeSystemAnalysis drakeSystemPrimitives)

  drake_add_cc_test(schunk_wsg_lcm_test)
  target_link_libraries(schunk_wsg_lcm_test
    drakeSchunkWsg drakeSystemAnalysis)
endif()
