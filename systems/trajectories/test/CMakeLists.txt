add_library(drakeTrajectoryTestUtil trajectoryTestUtil.cpp)

add_executable(testPiecewisePolynomial testPiecewisePolynomial.cpp)
target_link_libraries(testPiecewisePolynomial drakeTrajectories drakeTrajectoryTestUtil)
include_directories( .. )
include_directories (${CMAKE_SOURCE_DIR}/util/test)
add_test(NAME testPiecewisePolynomial COMMAND testPiecewisePolynomial)

add_executable(testExponentialPlusPiecewisePolynomial testExponentialPlusPiecewisePolynomial.cpp)
target_link_libraries(testExponentialPlusPiecewisePolynomial drakeTrajectories drakeTrajectoryTestUtil)
include_directories( .. )
include_directories (${CMAKE_SOURCE_DIR}/util/test)
add_test(NAME testExponentialPlusPiecewisePolynomial COMMAND testExponentialPlusPiecewisePolynomial)
