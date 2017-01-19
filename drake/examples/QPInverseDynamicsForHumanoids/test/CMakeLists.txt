drake_add_cc_test(NAME setpoint_test)
target_link_libraries(setpoint_test
    drakeQPController)

if(lcm_FOUND AND yaml-cpp_FOUND)
  drake_add_cc_test(NAME lcm_utils_test)
  target_link_libraries(lcm_utils_test
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerLcmTranslator)
endif()

if(gurobi_FOUND AND yaml-cpp_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerParamParsing
    drakeRigidBodyAliasGroups
    drakeValkyrieConstants)
endif()
