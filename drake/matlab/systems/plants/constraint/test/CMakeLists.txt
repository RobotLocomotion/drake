drake_add_matlab_test(NAME systems/plants/constraint/test/RelativeGazeTargetConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeGazeTargetConstraintTest SIZE large)
drake_add_matlab_test(NAME systems/plants/constraint/test/RelativePositionConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativePositionConstraintTest)
drake_add_matlab_test(NAME systems/plants/constraint/test/RelativeQuatConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND RelativeQuatConstraintTest SIZE large)
drake_add_matlab_test(NAME systems/plants/constraint/test/WorldPositionInFrameConstraintTest REQUIRES lcm libbot OPTIONAL bullet COMMAND WorldPositionInFrameConstraintTest)

# drake_add_matlab_test(NAME systems/plants/constraint/test/testKinCnst OPTIONAL bullet COMMAND testKinCnst)  # FIXME: see #2339

