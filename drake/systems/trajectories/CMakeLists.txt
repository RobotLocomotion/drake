if (MATLAB_FOUND)
  add_mex(PPTmex PPTmex.cpp)
  add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)
endif()

include_directories(${PROJECT_SOURCE_DIR}/util)
add_library(drakeTrajectories PiecewiseFunction.cpp PiecewisePolynomialBase.cpp PiecewisePolynomial.cpp ExponentialPlusPiecewisePolynomial.cpp)
target_link_libraries(drakeTrajectories drakePolynomial)
pods_install_libraries(drakeTrajectories)
pods_install_headers(PiecewiseFunction.h PiecewisePolynomial.h PiecewisePolynomialBase.h ExponentialPlusPiecewisePolynomial.h DESTINATION drake)
pods_install_pkg_config_file(drakeTrajectories
LIBS -ldrakeTrajectories
REQUIRES
VERSION 0.0.3)

add_subdirectory(test)
