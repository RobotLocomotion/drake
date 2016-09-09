if(MATLAB_FOUND)
  add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeJoints drakeMexUtil)
endif()

add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison
  COMMAND testDrakeJointsComparison)

add_executable(joint_test joint_test.cc)
target_link_libraries(joint_test drakeJoints ${GTEST_BOTH_LIBRARIES})
add_test(NAME joint_test COMMAND joint_test)
