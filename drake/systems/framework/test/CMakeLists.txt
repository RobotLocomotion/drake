if(GTEST_FOUND)
  add_executable(value_test value_test.cc)
  target_link_libraries(value_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
  add_test(NAME value_test COMMAND value_test)

  add_executable(basic_vector_test basic_vector_test.cc)
  target_link_libraries(basic_vector_test drakeSystemFramework drakePolynomial
    drakeCore ${GTEST_BOTH_LIBRARIES})
  add_test(NAME basic_vector_test COMMAND basic_vector_test)

  add_executable(basic_state_vector_test basic_state_vector_test.cc)
  target_link_libraries(basic_state_vector_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME basic_state_vector_test COMMAND basic_state_vector_test)

  add_executable(state_subvector_test state_subvector_test.cc)
  target_link_libraries(state_subvector_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME state_subvector_test COMMAND state_subvector_test)

  add_executable(state_test state_test.cc)
  target_link_libraries(state_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME state_test COMMAND state_test)

  add_executable(continuous_system_test continuous_system_test.cc)
  target_link_libraries(continuous_system_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME continuous_system_test COMMAND continuous_system_test)

  add_executable(system_input_test system_input_test.cc)
  target_link_libraries(system_input_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME system_input_test COMMAND system_input_test)

  add_executable(system_output_test system_output_test.cc)
  target_link_libraries(system_output_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME system_output_test COMMAND system_output_test)

  add_executable(context_test context_test.cc)
  target_link_libraries(context_test drakeSystemFramework
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME context_test COMMAND context_test)
endif()
