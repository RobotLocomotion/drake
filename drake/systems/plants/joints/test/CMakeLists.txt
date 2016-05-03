if (MATLAB_FOUND)
  add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeMexUtil drakeGeometryUtil)
endif(MATLAB_FOUND)

add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison COMMAND testDrakeJointsComparison)

add_executable(testJoint testJoint.cpp)
target_link_libraries(testJoint drakeGeometryUtil)
