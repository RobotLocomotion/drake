drake_add_cc_test(setpoint_test)
target_link_libraries(setpoint_test drakeQPController)

if(lcm_FOUND)
  drake_add_cc_test(lcm_utils_test)
  target_link_libraries(lcm_utils_test
    drakeMultibodyParsers
    drakeQPControllerLcmTranslator
    drakeQPController)
endif()

if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeMultibodyParsers
    drakeQPController)
endif()

if(gurobi_FOUND AND lcm_FOUND)
  add_executable(valkyrie_qp_inverse_dynamics_controller
    valkyrie_qp_inverse_dynamics_controller.cc)
  target_link_libraries(valkyrie_qp_inverse_dynamics_controller
    drakeLCMSystem2
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeRigidBodyPlant
    drakeSystemAnalysis)
endif()

