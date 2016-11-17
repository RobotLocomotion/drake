drake_add_cc_test(NAME setpoint_test)
target_link_libraries(setpoint_test
  drakeQPController
  GTest::GTest GTest::Main)

if(lcm_FOUND)
  drake_add_cc_test(NAME lcm_utils_test)
  target_link_libraries(lcm_utils_test
    drakeQPControllerLcmTranslator
    drakeQPController
    GTest::GTest GTest::Main)
endif()

drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeQPController
    GTest::GTest GTest::Main)
endif()

if(gurobi_FOUND AND lcm_FOUND)
  add_executable(val_sim val_sim.cc)
  target_link_libraries(val_sim
    drakeLCMSystem2
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeRigidBodyPlant
    drakeSystemAnalysis)

  add_executable(val_con val_con.cc)
  target_link_libraries(val_con
    drakeLCMSystem2
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeRigidBodyPlant
    drakeSystemAnalysis)
endif()

