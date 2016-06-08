if(GTEST_FOUND)
  add_executable(ros_test ros_test.cc)
  target_link_libraries(ros_test ${GTEST_BOTH_LIBRARIES} ${roscpp_LIBRARIES})
  # The following test is disabled because running it directly requires that
  # roscore be started separately.
  # add_test(NAME ros_test COMMAND ros_test)
endif()
