if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_wsg_system_test)
  target_link_libraries(simulated_schunk_wsg_system_test
    drakeSimulatedSchunkWsg drakeSystemAnalysis drakeSystemPrimitives)

  # With the new contact model (and current configuration values), this test
  # takes about 2.5 minutes to run in release mode.  It should *not*
  # be run in debug mode (> 90 minutes).
  drake_add_cc_test(NAME schunk_wsg_lift_test SIZE large
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
