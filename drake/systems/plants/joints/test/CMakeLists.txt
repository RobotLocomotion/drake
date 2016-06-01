if(MATLAB_FOUND)
  add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeJoints drakeMexUtil)
endif()

add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison COMMAND testDrakeJointsComparison)
