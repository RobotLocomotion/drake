# tk 20141113 debugMexTest causes compilation errors on Ubuntu 32 bit
# See https://github.com/RobotLocomotion/drake/issues/553
# debugMexTester.m and debugMexTest.cpp were moved to dev
# add_mex(debugMexTest debugMexTest.cpp)

include_directories(..)
include_directories(${PROJECT_SOURCE_DIR}/systems/trajectories)

add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
target_link_libraries(testDrakeGeometryUtil drakeGeometryUtil)
add_test(NAME testDrakeGeometryUtil COMMAND testDrakeGeometryUtil)

add_executable(testGradientVar testGradientVar.cpp)
#add_test(NAME testGradientVar COMMAND testGradientVar)

add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
target_link_libraries(testDrakeGradientUtil drakeGeometryUtil)
add_test(NAME testDrakeGradientUtil COMMAND testDrakeGradientUtil)

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

add_executable(testConvexHull testConvexHull.cpp)
target_link_libraries(testConvexHull drakeConvexHull)
add_test(NAME testConvexHull COMMAND testConvexHull)

add_executable(testPolynomial testPolynomial.cpp)
target_link_libraries(testPolynomial drakePolynomial)
add_test(NAME testPolynomial COMMAND testPolynomial)

add_executable(testTrigPoly testTrigPoly.cpp)
target_link_libraries(testTrigPoly drakePolynomial)
add_test(NAME testTrigPoly COMMAND testTrigPoly)
