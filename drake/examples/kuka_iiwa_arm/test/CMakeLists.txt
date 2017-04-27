if(lcm_FOUND)
  drake_add_cc_test(iiwa_lcm_test)
  target_link_libraries(iiwa_lcm_test
    drakeKukaIiwaArmCommon
    drakeLCMTypes
    Eigen3::Eigen)

  drake_add_cc_test(iiwa_plan_source_test)
  target_link_libraries(iiwa_plan_source_test drakeKukaIiwaArmCommon)

  drake_add_cc_test(sim_diagram_builder_test)
  target_link_libraries(sim_diagram_builder_test
      drakeKukaIiwaArmCommon
      drakeKukaIiwaWorld
      drakeSimDiagramBuilder
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers)
endif()
