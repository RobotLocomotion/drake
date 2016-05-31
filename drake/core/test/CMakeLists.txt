if(GTEST_FOUND)
  add_executable(vector_test vector_test.cc)
  target_link_libraries(vector_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME vector_test COMMAND vector_test)

  add_executable(functional_form_test functional_form_test.cc)
  target_link_libraries(functional_form_test drakeCore ${GTEST_BOTH_LIBRARIES})
  add_test(NAME functional_form_test COMMAND functional_form_test)
endif()
