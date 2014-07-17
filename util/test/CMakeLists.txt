
add_mex(debugMexTest debugMexTest.cpp)

if (eigen3_FOUND)
  add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
  include_directories( .. )
endif()