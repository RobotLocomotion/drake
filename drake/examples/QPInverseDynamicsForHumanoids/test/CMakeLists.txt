if(gurobi_FOUND)
  drake_add_cc_test(NAME valkyrie_balancing_test SIZE medium)
  target_link_libraries(valkyrie_balancing_test drakeQPController)
endif()

drake_add_cc_test(NAME setpoint_test)
target_link_libraries(setpoint_test drakeQPController)
