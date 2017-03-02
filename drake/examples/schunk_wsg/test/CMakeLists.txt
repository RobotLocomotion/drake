if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_wsg_system_test)
  target_link_libraries(simulated_schunk_wsg_system_test
    drakeSchunkWsg drakeSystemAnalysis drakeSystemPrimitives)

  drake_add_cc_test(schunk_wsg_lcm_test)
  target_link_libraries(schunk_wsg_lcm_test
    drakeSchunkWsg drakeSystemAnalysis)

  # With the new contact model (and current configuration values), this test
  # takes about 2.5 minutes to run in release mode.  It should *not*
  # be run in debug mode (> 90 mintes).
  drake_add_cc_test(NAME schunk_wsg_lift_test SIZE medium
    CONFIGURATIONS Release)
  target_link_libraries(schunk_wsg_lift_test
    drakeCommon
    drakeLcmSystem
    drakeMultibodyParsers
    drakeRBM
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemControllers
    drakeSystemPrimitives)
endif()
