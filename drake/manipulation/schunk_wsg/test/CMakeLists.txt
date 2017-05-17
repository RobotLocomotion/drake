if(lcm_FOUND)
  drake_add_cc_test(schunk_wsg_lcm_test)
  target_link_libraries(schunk_wsg_lcm_test
    drakeSchunkWsg drakeSystemAnalysis)
endif()

drake_add_cc_test(schunk_wsg_constants_test)
target_link_libraries(schunk_wsg_constants_test
  drakeMultibodyParsers
  drakeRBM)
