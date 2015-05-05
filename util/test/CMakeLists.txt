# tk 20141113 debugMexTest causes compilation errors on Ubuntu 32 bit
# See https://github.com/RobotLocomotion/drake/issues/553
# debugMexTester.m and debugMexTest.cpp were moved to dev
# add_mex(debugMexTest debugMexTest.cpp)

include_directories(..)
include_directories(${PROJECT_SOURCE_DIR}/systems/trajectories)
include_directories(${PROJECT_SOURCE_DIR}/solvers/polynomial)
include_directories(${PROJECT_BINARY_DIR}/lcmgen/lcmtypes)

if (NOT WIN32)
  # doesn't work because the timing tests require the <chrono> include
  add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
  target_link_libraries(testDrakeGeometryUtil drakeGeometryUtil drakeGradientUtil)
  add_test(NAME testDrakeGeometryUtil COMMAND testDrakeGeometryUtil)

  add_executable(testGradientVar testGradientVar.cpp)
  target_link_libraries(testGradientVar drakeGradientUtil)
  #set_target_properties(testGradientVar PROPERTIES COMPILE_FLAGS "-g -O0")
  #add_test(NAME testGradientVar COMMAND testGradientVar)

  add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
  target_link_libraries(testDrakeGradientUtil drakeGeometryUtil drakeGradientUtil)
  add_test(NAME testDrakeGradientUtil COMMAND testDrakeGradientUtil)
endif (NOT WIN32)

if (MATLAB_FOUND)
  add_mex(testQuatmex testQuatmex.cpp)
  target_link_libraries(testQuatmex drakeGeometryUtil)

  add_mex(testGeometryGradientsmex testGeometryGradientsmex.cpp)
  target_link_libraries(testGeometryGradientsmex drakeGeometryUtil drakeMexUtil)

  add_mex(testGeometryConversionFunctionsmex testGeometryConversionFunctionsmex.cpp)
  target_link_libraries(testGeometryConversionFunctionsmex drakeGeometryUtil drakeMexUtil)
endif(MATLAB_FOUND)

if (LCM_FOUND)
add_executable(testLCMUtil testLCMUtil.cpp)
target_link_libraries(testLCMUtil drakeLCMUtil)
endif()
