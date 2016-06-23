add_executable(autodiff_test autodiff_test.cc)
target_link_libraries(autodiff_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME autodiff_test COMMAND autodiff_test)

add_executable(expmap_test expmap_test.cc)
target_link_libraries(expmap_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME expmap_test COMMAND expmap_test)
