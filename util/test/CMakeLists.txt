
add_mex(debugMexTest debugMexTest.cpp)

if (eigen3_FOUND)
  if (NOT WIN32) # todo: fix this
  add_executable(testDrakeGeometryUtil testDrakeGeometryUtil.cpp)
  target_link_libraries(testDrakeGeometryUtil drakeGeometryUtil drakeGradientUtil)
  include_directories( .. )
  add_test(NAME testDrakeGeometryUtil COMMAND testDrakeGeometryUtil)

  add_executable(testDrakeGradientUtil testDrakeGradientUtil.cpp)
  target_link_libraries(testDrakeGradientUtil drakeGeometryUtil drakeGradientUtil)
  include_directories( .. )
  add_test(NAME testDrakeGradientUtil COMMAND testDrakeGradientUtil)

  endif(NOT WIN32)
endif(eigen3_FOUND)
