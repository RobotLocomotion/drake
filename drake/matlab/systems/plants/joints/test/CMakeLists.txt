if(Matlab_FOUND)
  drake_add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeJoints drakeMexUtil)
endif()

drake_add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison COMMAND testDrakeJointsComparison)

