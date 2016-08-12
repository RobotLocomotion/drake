add_executable(adder_test adder_test.cc)
target_link_libraries(adder_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder_test COMMAND adder_test)

add_executable(integrator_test integrator_test.cc)
target_link_libraries(integrator_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME integrator_test COMMAND integrator_test)

add_executable(dummy_test dummy_test.cc)
target_link_libraries(dummy_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME dummy_test COMMAND dummy_test)

add_executable(gain_test gain_test.cc)
target_link_libraries(gain_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME gain_test COMMAND gain_test)
