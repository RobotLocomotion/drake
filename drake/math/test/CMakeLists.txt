add_executable(autodiff_test autodiff_test.cc)
target_link_libraries(autodiff_test ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME autodiff_test COMMAND autodiff_test)

add_executable(expmap_test expmap_test.cc)
target_link_libraries(expmap_test ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME expmap_test COMMAND expmap_test)

add_executable(rotation_conversion_test rotation_conversion_test.cc)
target_link_libraries(rotation_conversion_test ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME rotation_conversion_test COMMAND rotation_conversion_test)

add_executable(cross_product_test cross_product_test.cc)
target_link_libraries(cross_product_test ${GTEST_BOTH_LIBRARIES})
drake_add_test(NAME cross_product_test COMMAND cross_product_test)
