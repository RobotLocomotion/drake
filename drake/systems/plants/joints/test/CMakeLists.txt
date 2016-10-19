drake_add_cc_test(joint_test)
target_link_libraries(joint_test drakeJoints)

# ==== Below this line is MATLAB-specific code ====

if(Matlab_FOUND)
  drake_add_mex(testDrakeJointsmex testDrakeJointsmex.cpp)
  target_link_libraries(testDrakeJointsmex drakeJoints drakeMexUtil)
endif()

drake_add_matlab_test(NAME systems/plants/joints/test/testDrakeJointsComparison COMMAND testDrakeJointsComparison)

# ==== Below this line is C++ and MATLAB shared code ====
