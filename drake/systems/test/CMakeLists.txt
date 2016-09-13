add_executable(testAffineSystem testAffineSystem.cpp)
drake_add_test(NAME testAffineSystem COMMAND testAffineSystem)
target_link_libraries(testAffineSystem drakeCommon ${GTEST_BOTH_LIBRARIES})

add_executable(cascade_system_test cascade_system_test.cc)
target_link_libraries(cascade_system_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME cascade_system_test COMMAND cascade_system_test)

add_executable(feedback_system_test feedback_system_test.cc)
target_link_libraries(feedback_system_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME feedback_system_test COMMAND feedback_system_test)

add_executable(n_ary_state_test n_ary_state_test.cc)
target_link_libraries(n_ary_state_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME n_ary_state_test COMMAND n_ary_state_test)

add_executable(n_ary_system_test n_ary_system_test.cc)
target_link_libraries(n_ary_system_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME n_ary_system_test COMMAND n_ary_system_test)

add_executable(simulation_termination_test simulation_termination_test.cc)
target_link_libraries(simulation_termination_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME simulation_termination_test COMMAND simulation_termination_test)

add_executable(vector_test vector_test.cc)
target_link_libraries(vector_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME vector_test COMMAND vector_test)

add_executable(simulation_trajectory_logger_test simulation_trajectory_logger_test.cc)
target_link_libraries(simulation_trajectory_logger_test drakeCommon ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME simulation_trajectory_logger_test COMMAND simulation_trajectory_logger_test)

drake_add_matlab_test(NAME systems/test/HybridSystemSampleTimeTest.run COMMAND HybridSystemSampleTimeTest.run)
drake_add_matlab_test(NAME systems/test/LinearGaussianDT.run COMMAND LinearGaussianDT.run)
# drake_add_matlab_test(NAME systems/test/linear_gaussian_dt COMMAND linear_gaussian_dt)  # FIXME: see #366
drake_add_matlab_test(NAME systems/test/memoryTest OPTIONAL bullet COMMAND memoryTest)
# drake_add_matlab_test(NAME systems/test/runPendulum COMMAND runPendulum)  # FIXME: see #310
drake_add_matlab_test(NAME systems/test/testSimInput COMMAND testSimInput)
