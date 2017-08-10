drake_add_cc_test(decision_variable_test)
target_link_libraries(decision_variable_test drakeOptimization)

drake_add_cc_test(indeterminate_test)
target_link_libraries(indeterminate_test drakeOptimization)

drake_add_cc_test(binding_test)
target_link_libraries(binding_test drakeOptimization)

drake_add_cc_test(mathematical_program_test)
target_link_libraries(mathematical_program_test drakeOptimization drakeCommon)

drake_add_cc_test(moby_lcp_solver_test)
target_link_libraries(moby_lcp_solver_test drakeOptimization)

drake_add_cc_test(NAME system_identification_test SIZE large)
target_link_libraries(system_identification_test drakeOptimization)

drake_add_cc_test(constraint_test)
target_link_libraries(constraint_test drakeOptimization drakeCommon)

drake_add_cc_test(cost_test)
target_link_libraries(cost_test drakeOptimization drakeCommon)

set(drakeOptimizationTestSources
        add_solver_util.cc
        optimization_examples.cc
        linear_program_examples.cc
        quadratic_program_examples.cc
        mathematical_program_test_util.cc)
set(drakeOptimizationHeaders
        add_solver_util.h
        optimization_examples.h
        linear_program_examples.h
        quadratic_program_examples.h
        mathematical_program_test_util.h
        generic_trivial_costs.h)
add_library(drakeOptimizationTest STATIC
    ${drakeOptimizationTestSources} ${drakeOptimizationHeaders})
target_link_libraries(drakeOptimizationTest
        Eigen3::Eigen
        gflags
        GTest::GTest
        drakeOptimization)

drake_add_cc_test(convex_optimization_test)
target_link_libraries(convex_optimization_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(equality_constrained_qp_solver_test)
target_link_libraries(equality_constrained_qp_solver_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(nonlinear_program_test)
target_link_libraries(nonlinear_program_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(linear_system_solver_test)
target_link_libraries(linear_system_solver_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(mixed_integer_optimization_test)
target_link_libraries(mixed_integer_optimization_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(gurobi_solver_test)
target_link_libraries(gurobi_solver_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(mosek_solver_test)
target_link_libraries(mosek_solver_test drakeCommon drakeOptimizationTest)

drake_add_cc_test(linear_complementary_problem_test)
target_link_libraries(linear_complementary_problem_test drakeOptimization)

drake_add_cc_test(ipopt_solver_test)
target_link_libraries(ipopt_solver_test drakeOptimizationTest)

drake_add_cc_test(snopt_solver_test)
target_link_libraries(snopt_solver_test drakeOptimizationTest)

drake_add_cc_test(solver_id_test)
target_link_libraries(solver_id_test drakeOptimization)

drake_add_cc_test(solver_type_converter_test)
target_link_libraries(solver_type_converter_test drakeOptimization)

