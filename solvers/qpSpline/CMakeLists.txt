if (eigen3_FOUND)
  add_library(drakeSplineGeneration splineGeneration.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
  target_link_libraries(drakeSplineGeneration drakePolynomial drakePiecewisePolynomial)
  pods_install_libraries(drakeSplineGeneration)
  pods_install_headers(splineGeneration.h ValueConstraint.h ContinuityConstraint.h DESTINATION drake)
  pods_install_pkg_config_file(drakeSplineGeneration
    LIBS -ldrakeSplineGeneration
    REQUIRES
    VERSION 0.0.2)

  add_mex(twoWaypointCubicSplinemex twoWaypointCubicSplinemex.cpp)
  include_directories (${CMAKE_SOURCE_DIR}/util)
  include_directories (${CMAKE_SOURCE_DIR}/solvers/polynomial)
  include_directories (${CMAKE_SOURCE_DIR}/systems/trajectories)
  target_link_libraries(twoWaypointCubicSplinemex drakeSplineGeneration drakeUtil)
  install(TARGETS twoWaypointCubicSplinemex DESTINATION matlab)

  add_mex(twoWaypointCubicSplineFreeKnotTimesmex twoWaypointCubicSplineFreeKnotTimesmex.cpp)
  include_directories (${CMAKE_SOURCE_DIR}/util)
  include_directories (${CMAKE_SOURCE_DIR}/solvers/polynomial)
  include_directories (${CMAKE_SOURCE_DIR}/systems/trajectories)
  target_link_libraries(twoWaypointCubicSplineFreeKnotTimesmex drakeSplineGeneration drakeUtil)
  install(TARGETS twoWaypointCubicSplineFreeKnotTimesmex DESTINATION matlab)

  add_subdirectory(test)
endif()
