add_executable(adder_test adder_test.cc)
target_link_libraries(adder_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder_test COMMAND adder_test)

add_executable(integrator_test integrator_test.cc)
target_link_libraries(integrator_test
        drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME integrator_test COMMAND integrator_test)

add_executable(constant_vector_source_test constant_vector_source_test.cc)
target_link_libraries(constant_vector_source_test
        drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME constant_vector_source_test COMMAND constant_vector_source_test)
