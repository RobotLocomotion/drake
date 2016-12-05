drake_add_cc_test(NAME setpoint_test)
target_link_libraries(setpoint_test
  drakeQPController
  GTest::GTest GTest::Main)

if(lcm_FOUND)
  drake_add_cc_test(NAME lcm_utils_test)
  target_link_libraries(lcm_utils_test
    drakeExamplesCommon
    drakeQPControllerLcmTranslator
    drakeQPController
    GTest::GTest GTest::Main)
endif()

drake_find_package(gurobi PKG_CONFIG)
if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeExamplesCommon
    drakeQPController
    GTest::GTest GTest::Main)
endif()

if(gurobi_FOUND AND lcm_FOUND)
  add_executable(valkyrie_qp_inverse_dynamics_controller
    valkyrie_qp_inverse_dynamics_controller.cc)
  target_link_libraries(valkyrie_qp_inverse_dynamics_controller
    drakeExamplesCommon
    drakeLCMSystem2
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeRigidBodyPlant
    drakeSystemAnalysis)
endif()

