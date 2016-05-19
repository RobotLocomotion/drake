# Build tests for Drake common utilities.
#
if (GTEST_FOUND)
  add_executable(nice_type_name_test nice_type_name_test.cc)
  target_link_libraries(nice_type_name_test drakeCommon
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME nice_type_name_test COMMAND nice_type_name_test)

  add_executable(unique_ptr_ref_test unique_ptr_ref_test.cc)
  target_link_libraries(unique_ptr_ref_test drakeCommon
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME unique_ptr_ref_test COMMAND unique_ptr_ref_test)
endif()
