if(MATLAB_FOUND)
  add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeJoints drakeMexUtil)
endif()

add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison
  COMMAND testDrakeJointsComparison)

add_executable(test_joints test_joints.cc)
target_link_libraries(test_joints drakeJoints ${GTEST_BOTH_LIBRARIES})
add_test(NAME test_joints COMMAND test_joints)
