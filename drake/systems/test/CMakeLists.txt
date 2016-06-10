add_executable(testAffineSystem testAffineSystem.cpp)
add_test(NAME testAffineSystem COMMAND testAffineSystem)

if(GTEST_FOUND)
  add_executable(cascade_system_test cascade_system_test.cc)
  target_link_libraries(cascade_system_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME cascade_system_test COMMAND cascade_system_test)

  add_executable(feedback_system_test feedback_system_test.cc)
  target_link_libraries(feedback_system_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME feedback_system_test COMMAND feedback_system_test)

  add_executable(n_ary_state_test n_ary_state_test.cc)
  target_link_libraries(n_ary_state_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME n_ary_state_test COMMAND n_ary_state_test)

  add_executable(n_ary_system_test n_ary_system_test.cc)
  target_link_libraries(n_ary_system_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME n_ary_system_test COMMAND n_ary_system_test)
endif()

add_matlab_test(NAME systems/test/HybridSystemSampleTimeTest.run COMMAND HybridSystemSampleTimeTest.run)
add_matlab_test(NAME systems/test/LinearGaussianDT.run COMMAND LinearGaussianDT.run)
# add_matlab_test(NAME systems/test/linear_gaussian_dt COMMAND linear_gaussian_dt)  # FIXME: see #366
add_matlab_test(NAME systems/test/memoryTest COMMAND memoryTest)
# add_matlab_test(NAME systems/test/runPendulum COMMAND runPendulum)  # FIXME: see #310
add_matlab_test(NAME systems/test/testSimInput COMMAND testSimInput)
