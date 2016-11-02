drake_add_matlab_test(NAME examples/Atlas/test/multiRobotTest COMMAND multiRobotTest)

# Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasFastBackwardWalking REQUIRES gurobi lcm COMMAND runAtlasFastBackwardWalking)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasFastWalking REQUIRES gurobi lcm COMMAND runAtlasFastWalking)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasTurning REQUIRES gurobi lcm COMMAND runAtlasTurning)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasTurning270 REQUIRES gurobi lcm COMMAND runAtlasTurning270)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingNarrowStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingNarrowStairs)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingStairs)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingTestWraparound REQUIRES gurobi lcm COMMAND runAtlasWalkingTestWraparound)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingTiltedBlocks REQUIRES iris lcm mosek COMMAND runAtlasWalkingTiltedBlocks)
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingDownStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingDownStairs)

# See Github Issues #2553 #2538
# drake_add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingHeelSupport REQUIRES iris lcm mosek COMMAND runAtlasWalkingHeelSupport)

drake_add_matlab_test(NAME examples/Atlas/test/runDRCDrillTask OPTIONAL bullet snopt COMMAND runDRCDrillTask)
drake_add_matlab_test(NAME examples/Atlas/test/runDRCValveTask OPTIONAL bullet snopt COMMAND runDRCValveTask)
drake_add_matlab_test(NAME examples/Atlas/test/runDoubleAtlas OPTIONAL bullet gurobi snopt COMMAND runDoubleAtlas)
drake_add_matlab_test(NAME examples/Atlas/test/runHandContactSensorTest REQUIRES snopt OPTIONAL bullet gurobi COMMAND runHandContactSensorTest)
drake_add_matlab_test(NAME examples/Atlas/test/singleJointParameterEstimation OPTIONAL bullet COMMAND singleJointParameterEstimation)
drake_add_matlab_test(NAME examples/Atlas/test/testChangeRootLink OPTIONAL bullet COMMAND testChangeRootLink)
drake_add_matlab_test(NAME examples/Atlas/test/testCollisionVolumes REQUIRES bullet iris mosek OPTIONAL gurobi snopt COMMAND testCollisionVolumes)
drake_add_matlab_test(NAME examples/Atlas/test/testKin OPTIONAL bullet COMMAND testKin)
drake_add_matlab_test(NAME examples/Atlas/test/testKin2 OPTIONAL bullet COMMAND testKin2)
# drake_add_matlab_test(NAME examples/Atlas/test/testKneeSingularity REQUIRES iris lcm mosek COMMAND testKneeSingularity)  # FIXME: see #1603
drake_add_matlab_test(NAME examples/Atlas/test/testMex OPTIONAL bullet COMMAND testMex)
drake_add_matlab_test(NAME examples/Atlas/test/testOutputGradients OPTIONAL bullet gurobi snopt COMMAND testOutputGradients)
drake_add_matlab_test(NAME examples/Atlas/test/testRunningPlanner REQUIRES bullet lcm libbot snopt COMMAND testRunningPlanner)
