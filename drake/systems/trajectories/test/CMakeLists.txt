if (GTEST_FOUND)
  add_executable(testPiecewisePolynomial testPiecewisePolynomial.cpp)
  target_link_libraries(testPiecewisePolynomial drakeTrajectories ${GTEST_BOTH_LIBRARIES})
  add_test(NAME testPiecewisePolynomial COMMAND testPiecewisePolynomial)
endif (GTEST_FOUND)

add_executable(testExponentialPlusPiecewisePolynomial testExponentialPlusPiecewisePolynomial.cpp)
target_link_libraries(testExponentialPlusPiecewisePolynomial drakeTrajectories)
add_test(NAME testExponentialPlusPiecewisePolynomial COMMAND testExponentialPlusPiecewisePolynomial)
