add_executable(adder_test adder_test.cc)
target_link_libraries(adder_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder_test COMMAND adder_test)

add_executable(pass_through_test
        pass_through_test.cc
        pass_through_scalartype_test.cc)
target_link_libraries(pass_through_test drakeSystemFramework
        ${GTEST_BOTH_LIBRARIES})
add_test(NAME pass_through_test COMMAND pass_through_test)

add_executable(integrator_test integrator_test.cc)
target_link_libraries(integrator_test
        drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME integrator_test COMMAND integrator_test)

add_executable(constant_value_source_test constant_value_source_test.cc)
target_link_libraries(constant_value_source_test
        drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME constant_value_source_test COMMAND constant_value_source_test)

add_executable(constant_vector_source_test constant_vector_source_test.cc)
target_link_libraries(constant_vector_source_test
        drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME constant_vector_source_test COMMAND constant_vector_source_test)

add_executable(gain_test gain_test.cc gain_scalartype_test.cc)
target_link_libraries(gain_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME gain_test COMMAND gain_test)

add_executable(pid_controller_test pid_controller_test.cc)
target_link_libraries(pid_controller_test drakeSystemFramework
        ${GTEST_BOTH_LIBRARIES})
add_test(NAME pid_controller_test COMMAND pid_controller_test)

add_executable(demultiplexer_test demultiplexer_test.cc)
target_link_libraries(demultiplexer_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME demultiplexer_test COMMAND demultiplexer_test)
