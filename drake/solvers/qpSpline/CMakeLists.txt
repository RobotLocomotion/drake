include_directories (${PROJECT_SOURCE_DIR}/util)
include_directories (${PROJECT_SOURCE_DIR}/systems/trajectories)

add_library(drakeSplineGeneration splineGeneration.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
target_link_libraries(drakeSplineGeneration drakeTrajectories)
pods_install_libraries(drakeSplineGeneration)
pods_install_headers(splineGeneration.h ValueConstraint.h ContinuityConstraint.h SplineInformation.h DESTINATION drake)
pods_install_pkg_config_file(drakeSplineGeneration
  LIBS -ldrakeSplineGeneration
  REQUIRES
  VERSION 0.0.2)

if (MATLAB_FOUND)
  add_mex(nWaypointCubicSplinemex nWaypointCubicSplinemex.cpp)
  target_link_libraries(nWaypointCubicSplinemex drakeSplineGeneration drakeMexUtil)
  install(TARGETS nWaypointCubicSplinemex DESTINATION matlab)

  add_mex(nWaypointCubicSplineFreeKnotTimesmex nWaypointCubicSplineFreeKnotTimesmex.cpp)
  target_link_libraries(nWaypointCubicSplineFreeKnotTimesmex drakeSplineGeneration drakeMexUtil)
  install(TARGETS nWaypointCubicSplineFreeKnotTimesmex DESTINATION matlab)
endif()

add_subdirectory(test)
