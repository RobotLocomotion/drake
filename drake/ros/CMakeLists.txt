find_program(ROSTEST_EXECUTABLE NAMES rostest)

if(ROSTEST_EXECUTABLE)
  message(STATUS "Found rostest: ${ROSTEST_EXECUTABLE} (found version \"$ENV{ROS_DISTRO}\")")
else()
  message(FATAL_ERROR "Could NOT find rostest")
endif()

add_subdirectory(drake)
