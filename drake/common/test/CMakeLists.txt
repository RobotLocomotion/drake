# Build tests for Drake common utilities.
#
if (GTEST_FOUND)
  add_executable(nice_type_name_test nice_type_name_test.cc)
  target_link_libraries(nice_type_name_test drakeCommon
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME nice_type_name_test COMMAND nice_type_name_test)

  add_executable(have_intersection_test have_intersection_test.cc)
  target_link_libraries(have_intersection_test drakeCommon
                        ${GTEST_BOTH_LIBRARIES})
  add_test(NAME have_intersection_test COMMAND have_intersection_test)
endif()
