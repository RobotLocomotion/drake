add_executable(direct_collocation_constraint_test
  direct_collocation_constraint_test.cc)
target_link_libraries(direct_collocation_constraint_test
  drakeDynamicConstraint ${GTEST_BOTH_LIBRARIES})
add_test(NAME direct_collocation_constraint_test
  COMMAND direct_collocation_constraint_test)

add_matlab_test(NAME systems/plants/constraint/test/AllBodiesClosestDistanceConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND AllBodiesClosestDistanceConstraintTest)
# DISABLED due to #3149.
# add_matlab_test(NAME systems/plants/constraint/test/RelativeGazeTargetConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeGazeTargetConstraintTest)
add_matlab_test(NAME systems/plants/constraint/test/RelativePositionConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativePositionConstraintTest)
# DISABLED due to #3149.
# add_matlab_test(NAME systems/plants/constraint/test/RelativeQuatConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeQuatConstraintTest)
add_matlab_test(NAME systems/plants/constraint/test/WorldPositionInFrameConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND WorldPositionInFrameConstraintTest)
add_matlab_test(NAME systems/plants/constraint/test/testMultipleTimeLinearPostureConstraint OPTIONAL bullet COMMAND testMultipleTimeLinearPostureConstraint)
# DISABLED due to #3149.
# add_matlab_test(NAME systems/plants/constraint/test/testPostureConstraint OPTIONAL bullet COMMAND testPostureConstraint)
add_matlab_test(NAME systems/plants/constraint/test/testQuasiStaticConstraint OPTIONAL bullet COMMAND testQuasiStaticConstraint)
add_matlab_test(NAME systems/plants/constraint/test/testSingleTimeLinearPostureConstraint OPTIONAL bullet COMMAND testSingleTimeLinearPostureConstraint)

if(LONG_RUNNING_TESTS)
  # add_matlab_test(NAME systems/plants/constraint/test/testKinCnst OPTIONAL bullet COMMAND testKinCnst PROPERTIES TIMEOUT 1500)  # FIXME: see #2339
endif()
