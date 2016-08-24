add_executable(rbp_test rbp_test.cc)
target_link_libraries(rbp_test drake_rbp drakeRBSystem ${GTEST_BOTH_LIBRARIES})
add_test(NAME rbp_test COMMAND rbp_test)
