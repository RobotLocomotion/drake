# Build tests for Drake common utilities.
#
if (GTEST_FOUND)
  add_executable(test-nice_type_name test-nice_type_name.cc)
  target_link_libraries(test-nice_type_name drakeCommon
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME test-nice_type_name COMMAND test-nice_type_name)
endif()
