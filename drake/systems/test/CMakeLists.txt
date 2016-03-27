add_executable(testAffineSystem testAffineSystem.cpp)
add_test(NAME testAffineSystem COMMAND testAffineSystem)

if (GTEST_FOUND)
	add_executable(cascade_system_test cascade_system_test.cc)
	target_link_libraries(cascade_system_test ${GTEST_BOTH_LIBRARIES})
	add_test(NAME cascade_system_test COMMAND cascade_system_test)

	add_executable(feedback_system_test feedback_system_test.cc)
	target_link_libraries(feedback_system_test ${GTEST_BOTH_LIBRARIES})
	add_test(NAME feedback_system_test COMMAND feedback_system_test)
endif()
