add_library(splineGeneration splineGeneration.cpp Polynomial.cpp PiecewisePolynomialBase.cpp PiecewisePolynomial.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
pods_install_libraries(splineGeneration)
pods_install_headers(splineGeneration.h Polynomial.h PiecewisePolynomialBase.h PiecewisePolynomial.h SplineInformation.h ValueConstraint.h ContinuityConstraint.h DESTINATION qpSpline)
pods_install_pkg_config_file(qpSpline
  LIBS -lqpSpline
  REQUIRES
  VERSION 0.0.2)

add_mex(twoWaypointCubicSplinemex twoWaypointCubicSplinemex.cpp)
include_directories (${CMAKE_SOURCE_DIR}/util)
target_link_libraries(twoWaypointCubicSplinemex splineGeneration drakeUtil)
install(TARGETS twoWaypointCubicSplinemex DESTINATION matlab)

add_subdirectory(test)
