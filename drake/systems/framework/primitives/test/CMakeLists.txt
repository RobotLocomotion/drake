add_executable(adder_test adder_test.cc)
target_link_libraries(adder_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder_test COMMAND adder_test)

add_executable(integrator_test integrator_test.cc)
target_link_libraries(integrator_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME integrator_test COMMAND integrator_test)

add_executable(autodiff_alignment_test autodiff_alignment_test.cc)
target_link_libraries(autodiff_alignment_test ${GTEST_BOTH_LIBRARIES})
add_test(NAME autodiff_alignment_test COMMAND autodiff_alignment_test)
