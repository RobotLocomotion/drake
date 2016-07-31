add_executable(function_test function_test.cc)
target_link_libraries(function_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME function_test COMMAND function_test)

add_executable(vector_test vector_test.cc)
target_link_libraries(vector_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME vector_test COMMAND vector_test)
