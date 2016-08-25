add_executable(rbs_test rbs_test.cc)
target_link_libraries(rbs_test drake_rbs drakeRBSystem ${GTEST_BOTH_LIBRARIES})
add_test(NAME rbs_test COMMAND rbs_test)
