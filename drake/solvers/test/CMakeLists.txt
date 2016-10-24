drake_add_cc_test(mathematical_program_test)
target_link_libraries(mathematical_program_test drakeOptimization)

# TODO(hongkai.dai@tri.global) : re-enable mosek test after re-writing
# mosek_solver and the test
if(mosek_FOUND)
  add_executable(mosek_test mosek_test.cc)
  target_include_directories(mosek_test PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(mosek_test drakeOptimization ${GTEST_BOTH_LIBRARIES})
endif()

drake_add_cc_test(moby_lcp_solver_test)
target_link_libraries(moby_lcp_solver_test drakeOptimization)

drake_add_cc_test(NAME system_identification_test SIZE medium)
target_link_libraries(system_identification_test drakeOptimization)

drake_add_cc_test(NAME fastqp_solver_test)
target_link_libraries(fastqp_solver_test drakeOptimization)


drake_add_cc_test(constraint_test)
target_link_libraries(constraint_test drakeOptimization drakeCommon)

if(gurobi_FOUND)
  drake_add_cc_test(gurobi_solver_test)
  target_link_libraries(gurobi_solver_test drakeOptimization)
endif()

if(dreal_FOUND)
  drake_add_cc_test(dreal_solver_test)
  target_link_libraries(dreal_solver_test drakeOptimization)
endif()

if(gurobi_FOUND AND snopt_c_FOUND)
  drake_add_cc_test(gurobi_snopt_qp_test)
  target_link_libraries(gurobi_snopt_qp_test drakeOptimization)
endif()
