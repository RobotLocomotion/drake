
add_mex(debugMexTest debugMexTest.cpp)

if (eigen3_FOUND)
  add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
  include_directories( .. ../../../eigen/eigen-eigen-6b38706d90a9/unsupported ) # TODO: get rid of unsupported
  target_link_libraries(testDrakeGradientUtil drakeQuatUtil)
  
  add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
  include_directories( .. )
endif()