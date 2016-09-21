add_executable(optimization_problem_test mathematical_program_test.cc)
target_link_libraries(optimization_problem_test drakeOptimization
  ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME optimization_problem_test COMMAND optimization_problem_test)

if(mosek_FOUND)
  add_executable(mosek_test mosek_test.cc)
  target_link_libraries(mosek_test drakeOptimization ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME mosek_test COMMAND mosek_test)
endif()

add_executable(moby_lcp_solver_test moby_lcp_solver_test.cc)
target_link_libraries(moby_lcp_solver_test drakeOptimization ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME moby_lcp_solver_test COMMAND moby_lcp_solver_test)

add_executable(system_identification_test system_identification_test.cc)
target_link_libraries(system_identification_test
  drakeOptimization drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME system_identification_test COMMAND system_identification_test SIZE medium)
add_executable(gurobi_solver_test gurobi_solver_test.cc)
target_link_libraries(gurobi_solver_test drakeOptimization
  ${GTEST_BOTH_LIBRARIES})

if(gurobi_FOUND)
  drake_add_test(NAME gurobi_solver_test COMMAND gurobi_solver_test)
endif()

if(dreal_FOUND)
  add_executable(dreal_solver_test dreal_solver_test.cc)
  target_link_libraries(dreal_solver_test drakeOptimization
    ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME dreal_solver_test COMMAND dreal_solver_test)
endif()
