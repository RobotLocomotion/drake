if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/Atlas/test/multiRobotTest COMMAND multiRobotTest PROPERTIES TIMEOUT 1500)

  # Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
  # add_matlab_test(NAME examples/Atlas/test/runAtlasFastBackwardWalking REQUIRES gurobi lcm COMMAND runAtlasFastBackwardWalking PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasFastWalking REQUIRES gurobi lcm COMMAND runAtlasFastWalking PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasTurning REQUIRES gurobi lcm COMMAND runAtlasTurning PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasTurning270 REQUIRES gurobi lcm COMMAND runAtlasTurning270 PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingNarrowStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingNarrowStairs PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingStairs PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingTestWraparound REQUIRES gurobi lcm COMMAND runAtlasWalkingTestWraparound PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingTiltedBlocks REQUIRES iris lcm mosek COMMAND runAtlasWalkingTiltedBlocks PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingDownStairs REQUIRES iris lcm mosek COMMAND runAtlasWalkingDownStairs PROPERTIES TIMEOUT 1500)

  # See Github Issues #2553 #2538
  # add_matlab_test(NAME examples/Atlas/test/runAtlasWalkingHeelSupport REQUIRES iris lcm mosek COMMAND runAtlasWalkingHeelSupport PROPERTIES TIMEOUT 1500)
endif()

add_matlab_test(NAME examples/Atlas/test/runDRCDrillTask OPTIONAL bullet snopt COMMAND runDRCDrillTask)
add_matlab_test(NAME examples/Atlas/test/runDRCValveTask OPTIONAL bullet snopt COMMAND runDRCValveTask)
add_matlab_test(NAME examples/Atlas/test/runDoubleAtlas OPTIONAL bullet gurobi snopt COMMAND runDoubleAtlas)
add_matlab_test(NAME examples/Atlas/test/runHandContactSensorTest OPTIONAL bullet gurobi snopt COMMAND runHandContactSensorTest)
add_matlab_test(NAME examples/Atlas/test/singleJointParameterEstimation OPTIONAL bullet COMMAND singleJointParameterEstimation)
add_matlab_test(NAME examples/Atlas/test/testChangeRootLink OPTIONAL bullet COMMAND testChangeRootLink)
add_matlab_test(NAME examples/Atlas/test/testCollisionVolumes REQUIRES bullet iris mosek OPTIONAL gurobi snopt COMMAND testCollisionVolumes)
add_matlab_test(NAME examples/Atlas/test/testKin OPTIONAL bullet COMMAND testKin)
add_matlab_test(NAME examples/Atlas/test/testKin2 OPTIONAL bullet COMMAND testKin2)
# add_matlab_test(NAME examples/Atlas/test/testKneeSingularity REQUIRES iris lcm mosek COMMAND testKneeSingularity)  # FIXME: see #1603
add_matlab_test(NAME examples/Atlas/test/testMex OPTIONAL bullet COMMAND testMex)
add_matlab_test(NAME examples/Atlas/test/testOutputGradients OPTIONAL bullet gurobi snopt COMMAND testOutputGradients)
add_matlab_test(NAME examples/Atlas/test/testRunningPlanner REQUIRES bullet lcm libbot OPTIONAL snopt COMMAND testRunningPlanner)
