add_executable(testAffineSystem testAffineSystem.cpp)
add_test(NAME testAffineSystem COMMAND testAffineSystem)

if (GTEST_FOUND)
	add_executable(cascade_system_test cascade_system_test.cc)
	target_link_libraries(cascade_system_test ${GTEST_BOTH_LIBRARIES})
	add_test(NAME cascade_system_test COMMAND cascade_system_test)

	add_executable(feedback_system_test feedback_system_test.cc)
	target_link_libraries(feedback_system_test ${GTEST_BOTH_LIBRARIES})
	add_test(NAME feedback_system_test COMMAND feedback_system_test)

    if (WITH_ROS)
		find_package(roscpp QUIET)
		if(NOT roscpp_FOUND)
			message(STATUS "ERROR: Couldn't find roscpp, so not building ROS-based unit test. Did you source the ROS setup.sh file?")
		else()
			message(STATUS "Found roscpp installed at ${roscpp_DIR}, so building ROS-based unit test.")
			include_directories(${roscpp_INCLUDE_DIRS})
			add_executable(ros_test ros_test.cc)
			target_link_libraries(ros_test ${GTEST_BOTH_LIBRARIES} ${roscpp_LIBRARIES})
			add_test(NAME ros_test COMMAND ros_test)
		endif()
	endif()
endif()

add_matlab_test(NAME systems/test/HybridSystemSampleTimeTest.run COMMAND HybridSystemSampleTimeTest.run)
add_matlab_test(NAME systems/test/LinearGaussianDT.run COMMAND LinearGaussianDT.run)
# add_matlab_test(NAME systems/test/linear_gaussian_dt COMMAND linear_gaussian_dt)  # see #366
add_matlab_test(NAME systems/test/memoryTest COMMAND memoryTest)
# add_matlab_test(NAME systems/test/runPendulum COMMAND runPendulum) # see #310
add_matlab_test(NAME systems/test/testSimInput COMMAND testSimInput)
