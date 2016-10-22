drake_add_cc_test(mathematical_program_test)
target_link_libraries(mathematical_program_test drakeOptimization)

add_library_with_exports(LIB_NAME drakeOptimizationExamples SOURCE_FILES optimization_program_examples.cc)
target_link_libraries(drakeOptimizationExamples drakeOptimization ${GTEST_BOTH_LIBRARIES})
drake_install_headers(optimization_program_examples.h)
pods_install_libraries(drakeOptimizationExamples)
pods_install_pkg_config_file(drake-optimization-examples
    LIBS -ldrakeOptimizationExamples
    REQUIRES
    VERSION 0.0.1)

if(mosek_FOUND)
  drake_add_cc_test(mosek_solver_test)
  target_link_libraries(mosek_solver_test drakeOptimization drakeOptimizationExamples)
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
