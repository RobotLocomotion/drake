add_executable(testAffineSystem testAffineSystem.cpp)
drake_add_test(NAME testAffineSystem COMMAND testAffineSystem)
target_link_libraries(testAffineSystem drakeCommon ${GTEST_BOTH_LIBRARIES})

drake_add_cc_test(cascade_system_test)
target_link_libraries(cascade_system_test drakeCommon)

drake_add_cc_test(feedback_system_test)
target_link_libraries(feedback_system_test drakeCommon)

drake_add_cc_test(simulation_termination_test)
target_link_libraries(simulation_termination_test drakeCommon)

drake_add_cc_test(vector_test)
target_link_libraries(vector_test drakeCommon)

drake_add_cc_test(simulation_trajectory_logger_test)
target_link_libraries(simulation_trajectory_logger_test drakeCommon)

drake_add_matlab_test(NAME systems/test/HybridSystemSampleTimeTest.run COMMAND HybridSystemSampleTimeTest.run)
drake_add_matlab_test(NAME systems/test/LinearGaussianDT.run COMMAND LinearGaussianDT.run)
# drake_add_matlab_test(NAME systems/test/linear_gaussian_dt COMMAND linear_gaussian_dt)  # FIXME: see #366
drake_add_matlab_test(NAME systems/test/memoryTest OPTIONAL bullet COMMAND memoryTest)
# drake_add_matlab_test(NAME systems/test/runPendulum COMMAND runPendulum)  # FIXME: see #310
drake_add_matlab_test(NAME systems/test/testSimInput COMMAND testSimInput)
