if (MATLAB_FOUND)
  add_mex(PPTmex PPTmex.cpp)
  add_mex(ExpPlusPPTrajectoryEvalmex ExpPlusPPTrajectoryEvalmex.cpp)
endif()

add_library(drakePiecewisePolynomial PiecewisePolynomialBase.cpp PiecewisePolynomial.cpp)
include_directories(${CMAKE_SOURCE_DIR}/solvers/polynomial)
target_link_libraries(drakePiecewisePolynomial drakePolynomial)
pods_install_libraries(drakePiecewisePolynomial)
pods_install_headers(PiecewisePolynomial.h PiecewisePolynomialBase.h DESTINATION drake)
pods_install_pkg_config_file(drakePiecewisePolynomial
LIBS -ldrakePiecewisePolynomial
REQUIRES
VERSION 0.0.2)

add_subdirectory(test)
