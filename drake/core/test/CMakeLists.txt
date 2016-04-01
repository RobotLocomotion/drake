if (GTEST_FOUND)
  add_executable(vector_test vector_test.cc)
  target_link_libraries(vector_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME vector_test COMMAND vector_test)

  add_executable(eigen_matrix_compare_test eigen_matrix_compare_test.cc)
  target_link_libraries(eigen_matrix_compare_test ${GTEST_BOTH_LIBRARIES})
  add_test(NAME eigen_matrix_compare_test COMMAND eigen_matrix_compare_test)
endif (GTEST_FOUND)
