drake_add_cc_test(decision_variable_test)
target_link_libraries(decision_variable_test drakeOptimization)

drake_add_cc_test(binding_test)
target_link_libraries(binding_test drakeOptimization)

drake_add_cc_test(mathematical_program_test)
target_link_libraries(mathematical_program_test drakeOptimization)

drake_add_cc_test(convex_optimization_test)
target_link_libraries(convex_optimization_test drakeOptimization)

drake_add_cc_test(moby_lcp_solver_test)
target_link_libraries(moby_lcp_solver_test drakeOptimization)

drake_add_cc_test(mixed_integer_optimization_test)
target_link_libraries(mixed_integer_optimization_test drakeOptimization)

drake_add_cc_test(NAME system_identification_test SIZE medium)
target_link_libraries(system_identification_test drakeOptimization)

drake_add_cc_test(constraint_test)
target_link_libraries(constraint_test drakeOptimization drakeCommon)

if(dreal_FOUND)
  drake_add_cc_test(dreal_solver_test)
  target_link_libraries(dreal_solver_test drakeOptimization)
endif()

if (mosek_FOUND)
  drake_add_cc_test(rotation_constraint_test rotation_constraint_test.cc)
  target_link_libraries(rotation_constraint_test drakeOptimization)
endif(mosek_FOUND)

if (lcm_FOUND)
  add_executable(plot_feasible_rotation_matrices plot_feasible_rotation_matrices.cc)
  target_link_libraries(plot_feasible_rotation_matrices 
    drakeOptimization 
    drakeLCMTypes
    drakeLcm)
endif(lcm_FOUND)