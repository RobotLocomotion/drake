drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND AND lcm_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_system_test SIZE medium)
  target_link_libraries(valkyrie_balancing_system_test
    drakeLCMSystem2
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeRigidBodyPlant
    drakeSystemAnalysis
    GTest::GTest GTest::Main)
endif()
