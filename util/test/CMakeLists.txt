# tk 20141113 debugMexTest causes compilation errors on Ubuntu 32 bit
# See https://github.com/RobotLocomotion/drake/issues/553
# debugMexTester.m and debugMexTest.cpp were moved to dev
# add_mex(debugMexTest debugMexTest.cpp)

if (eigen3_FOUND)
  if (NOT WIN32)
  # doesn't work because the timing tests require the <chrono> include
  add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
  target_link_libraries(testDrakeGeometryUtil drakeGeometryUtil drakeGradientUtil)
  include_directories( .. )
  add_test(NAME testDrakeGeometryUtil COMMAND testDrakeGeometryUtil)

  add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
  target_link_libraries(testDrakeGradientUtil drakeGeometryUtil drakeGradientUtil)
  include_directories( .. )
  add_test(NAME testDrakeGradientUtil COMMAND testDrakeGradientUtil)
  endif (NOT WIN32)
  
  add_mex(testQuatmex testQuatmex.cpp)
  target_link_libraries(testQuatmex drakeGeometryUtil)

  add_mex(testGeometryGradientsmex testGeometryGradientsmex.cpp)
  target_link_libraries(testGeometryGradientsmex drakeGeometryUtil)

  add_mex(testGeometryConversionFunctionsmex testGeometryConversionFunctionsmex.cpp)
  target_link_libraries(testGeometryConversionFunctionsmex drakeGeometryUtil)
endif(eigen3_FOUND)
