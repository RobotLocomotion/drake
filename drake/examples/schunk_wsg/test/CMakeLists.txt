if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_wsg_system_test)
  target_link_libraries(simulated_schunk_wsg_system_test
    drakeSchunkWsg drakeSystemAnalysis drakeSystemPrimitives)

  drake_add_cc_test(schunk_wsg_lcm_test)
  target_link_libraries(schunk_wsg_lcm_test
    drakeSchunkWsg drakeSystemAnalysis)

  # This test doesn't take too long in Release builds (about 20s,
  # which is consistent with other longer tests), but it's much much
  # slower in Debug.
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
