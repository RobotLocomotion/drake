add_library_with_exports(LIB_NAME drakeSplineGeneration SOURCE_FILES splineGeneration.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
target_link_libraries(drakeSplineGeneration drakeTrajectories)
pods_install_libraries(drakeSplineGeneration)
drake_install_headers(splineGeneration.h ValueConstraint.h ContinuityConstraint.h SplineInformation.h)
pods_install_pkg_config_file(drakeSplineGeneration
  LIBS -ldrakeSplineGeneration
  REQUIRES
  VERSION 0.0.2)

if (MATLAB_FOUND)
  add_mex(nWaypointCubicSplinemex nWaypointCubicSplinemex.cpp)
  target_link_libraries(nWaypointCubicSplinemex drakeSplineGeneration drakeMexUtil)

  add_mex(nWaypointCubicSplineFreeKnotTimesmex nWaypointCubicSplineFreeKnotTimesmex.cpp)
  target_link_libraries(nWaypointCubicSplineFreeKnotTimesmex drakeSplineGeneration drakeMexUtil)
endif()

add_subdirectory(test)
