add_executable(value_test value_test.cc)
target_link_libraries(value_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME value_test COMMAND value_test)

add_executable(basic_vector_test basic_vector_test.cc)
target_link_libraries(basic_vector_test drakeSystemFramework drakeCommon
  ${GTEST_BOTH_LIBRARIES})
add_test(NAME basic_vector_test COMMAND basic_vector_test)

add_executable(state_subvector_test state_subvector_test.cc)
target_link_libraries(state_subvector_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME state_subvector_test COMMAND state_subvector_test)

add_executable(state_supervector_test state_supervector_test.cc)
target_link_libraries(state_supervector_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME state_supervector_test COMMAND state_supervector_test)

add_executable(state_test state_test.cc)
target_link_libraries(state_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME state_test COMMAND state_test)

add_executable(system_test system_test.cc)
target_link_libraries(system_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME system_test COMMAND system_test)

add_executable(system_input_test system_input_test.cc)
target_link_libraries(system_input_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME system_input_test COMMAND system_input_test)

add_executable(system_output_test system_output_test.cc)
target_link_libraries(system_output_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME system_output_test COMMAND system_output_test)

add_executable(leaf_context_test leaf_context_test.cc)
target_link_libraries(leaf_context_test drakeSystemFramework
                      ${GTEST_BOTH_LIBRARIES})
add_test(NAME leaf_context_test COMMAND leaf_context_test)

add_executable(diagram_test diagram_test.cc)
target_link_libraries(diagram_test
                      drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME diagram_test COMMAND diagram_test)

add_executable(diagram_builder_test diagram_builder_test.cc)
target_link_libraries(diagram_builder_test
                      drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME diagram_builder_test COMMAND diagram_builder_test)

add_executable(diagram_context_test diagram_context_test.cc)
target_link_libraries(diagram_context_test
                      drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME diagram_context_test COMMAND diagram_context_test)
