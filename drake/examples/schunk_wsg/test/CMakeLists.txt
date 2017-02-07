if(lcm_FOUND)
  drake_add_cc_test(simulated_schunk_wsg_system_test)
  target_link_libraries(simulated_schunk_wsg_system_test
    drakeSchunkWsg drakeSystemAnalysis drakeSystemPrimitives)

  drake_add_cc_test(schunk_wsg_lcm_test)
  target_link_libraries(schunk_wsg_lcm_test
    drakeSchunkWsg drakeSystemAnalysis)

  drake_add_cc_test(NAME schunk_wsg_lift_test SIZE medium)
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
