
include_directories(${CMAKE_SOURCE_DIR}/util ${CMAKE_SOURCE_DIR}/solvers/polynomial)
add_mex(dynamicsRHS dynamicsRHS.cpp)
target_link_libraries(dynamicsRHS drakeUtil drakePolynomial)