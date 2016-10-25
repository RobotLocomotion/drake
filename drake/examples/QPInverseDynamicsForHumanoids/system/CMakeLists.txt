drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND AND lcm_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_system_test)
  target_link_libraries(valkyrie_balancing_system_test
    drakeQPController
    drakeSystemAnalysis
    drakeLCMSystem2
    drakeRigidBodyPlant
    drakeQPControllerLcmTranslator
    GTest::GTest GTest::Main)
endif()
