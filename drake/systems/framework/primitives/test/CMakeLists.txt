# System 2 primitive tests
add_executable(adder_test adder_test.cc)
target_link_libraries(adder_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder_test COMMAND adder_test)

# System 3 primitive tests
add_executable(adder3_test adder3_test.cc)
target_link_libraries(adder3_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME adder3_test COMMAND adder3_test)

add_executable(diagram3_test diagram3_test.cc)
target_link_libraries(diagram3_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME diagram3_test COMMAND diagram3_test)

add_executable(cascade3_test cascade3_test.cc)
target_link_libraries(cascade3_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME cascade3_test COMMAND cascade3_test)

add_executable(vector_constant3_test vector_constant3_test.cc)
target_link_libraries(vector_constant3_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME vector_constant3_test COMMAND vector_constant3_test)

add_executable(gain3_test gain3_test.cc)
target_link_libraries(gain3_test drakeSystemFramework ${GTEST_BOTH_LIBRARIES})
add_test(NAME gain3_test COMMAND gain3_test)
