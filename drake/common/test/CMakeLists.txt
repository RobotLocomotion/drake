# Build tests for Drake common utilities.
#
if(GTEST_FOUND)
  add_executable(nice_type_name_test nice_type_name_test.cc)
  target_link_libraries(nice_type_name_test drakeCommon ${GTEST_BOTH_LIBRARIES})
  add_test(NAME nice_type_name_test COMMAND nice_type_name_test)

  add_executable(drake_assert_test drake_assert_test.cc)
  target_link_libraries(drake_assert_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME drake_assert_test COMMAND drake_assert_test)
endif()
