add_executable(direct_collocation_constraint_test
  direct_collocation_constraint_test.cc)
target_link_libraries(direct_collocation_constraint_test
  drakeDynamicConstraint ${GTEST_BOTH_LIBRARIES})
add_test(NAME direct_collocation_constraint_test
  COMMAND direct_collocation_constraint_test)

# add_matlab_test(NAME systems/plants/constraint/test/RelativeGazeTargetConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeGazeTargetConstraintTest SIZE large)  # FIXME: see #3149
add_matlab_test(NAME systems/plants/constraint/test/AllBodiesClosestDistanceConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND AllBodiesClosestDistanceConstraintTest SIZE large)
add_matlab_test(NAME systems/plants/constraint/test/RelativePositionConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativePositionConstraintTest)
# add_matlab_test(NAME systems/plants/constraint/test/RelativeQuatConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeQuatConstraintTest SIZE large) # FIXME: see #3149
add_matlab_test(NAME systems/plants/constraint/test/WorldPositionInFrameConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND WorldPositionInFrameConstraintTest)
add_matlab_test(NAME systems/plants/constraint/test/testMultipleTimeLinearPostureConstraint OPTIONAL bullet COMMAND testMultipleTimeLinearPostureConstraint)
# add_matlab_test(NAME systems/plants/constraint/test/testPostureConstraint OPTIONAL bullet COMMAND testPostureConstraint SIZE large)  # FIXME: see #3149
add_matlab_test(NAME systems/plants/constraint/test/testQuasiStaticConstraint OPTIONAL bullet COMMAND testQuasiStaticConstraint)
add_matlab_test(NAME systems/plants/constraint/test/testSingleTimeLinearPostureConstraint OPTIONAL bullet COMMAND testSingleTimeLinearPostureConstraint)

# add_matlab_test(NAME systems/plants/constraint/test/testKinCnst OPTIONAL bullet COMMAND testKinCnst)  # FIXME: see #2339
