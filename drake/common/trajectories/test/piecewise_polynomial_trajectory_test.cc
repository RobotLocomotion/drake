#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

#include "gtest/gtest.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

using Eigen::MatrixXd;

namespace drake {
namespace systems {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

// Test all constructors. Try using both vector and matrix to construct.
// Verify that the matrix returned by value() gives the values we expect.
GTEST_TEST(piecewisePolynomialTrajectoryTest, testBasicFunctionality) {
  // Setup testing data
  const Polynomiald a = 3;
  const Polynomiald y = Polynomiald("y");
  const Polynomiald y2 = (2 * y);

  // Construct a vector of polynomials.
  const std::vector<Polynomiald> p_vec {a, y, y2};
  const std::vector<double> times {0.0, 3, 7, 9};  // Knot points.

  // Test the constructor that takes a PiecewisePolynomial (PP).
  // The PiecewisePolynomial can be either a vector of polynomials or a matrix
  // of polynomials.
  // Create a PiecewisePolynomial from a vector of polynomials, and use that
  // to construct a PiecewisePolynomialTrajectory.
  const PiecewisePolynomialType kPpFromVec(p_vec, times);

  // Create a PiecewisePolynomialTrajectory from a PP made from a vector.
  const PiecewisePolynomialTrajectory kPpTrajFromVec {kPpFromVec};
  EXPECT_EQ(kPpTrajFromVec.rows(), 1);
  EXPECT_EQ(kPpTrajFromVec.cols(), 1);
  // Test first segment 0 <= t < 3, returning the constant a=3.
  EXPECT_EQ(kPpTrajFromVec.value(0.0).value(), 3);
  EXPECT_EQ(kPpTrajFromVec.value(2).value(), 3);
  EXPECT_EQ(kPpTrajFromVec.value(2.9).value(), 3);
  // Test second segment, 3 =< t < 7. Must subtract the knot point 3 when
  // evaluating the variable.
  EXPECT_EQ(kPpTrajFromVec.value(3).value(), y.EvaluateUnivariate(3 - 3));
  EXPECT_EQ(kPpTrajFromVec.value(4)(0), y.EvaluateUnivariate(4 - 3));
  // Test third segment, 7 =< t < 9. Must subtract the knot point 7 when
  // evaluating the variable.
  EXPECT_EQ(kPpTrajFromVec.value(7)(0), y2.EvaluateUnivariate(7 - 7));
  EXPECT_EQ(kPpTrajFromVec.value(8)(0), y2.EvaluateUnivariate(8 - 7));
  EXPECT_EQ(kPpTrajFromVec.value(8.9)(0), y2.EvaluateUnivariate(8.9 - 7));

  // Test: construct a PiecewisePolynomialTrajectory from a PP matrix.

  // Construct a matrix of polynomials.
  std::vector<PiecewisePolynomialType::PolynomialMatrix> kPpMatrix(1);
  kPpMatrix[0].resize(3, 1);
  kPpMatrix[0](0) = a;
  kPpMatrix[0](1) = y;
  kPpMatrix[0](2) = y2;

  // Create a PiecewisePolynomial from a matrix.
  const PiecewisePolynomialType kPpFromMatrix(kPpMatrix, {0.0, 3});

  // Create a PiecewisePolynomialTrajectory from a PP made from a matrix.
  const PiecewisePolynomialTrajectory kPpTrajFromPpMatrix {kPpFromMatrix};
  EXPECT_EQ(kPpTrajFromPpMatrix.rows(), 3);
  EXPECT_EQ(kPpTrajFromPpMatrix.cols(), 1);
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2.9)(0), 3);
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2.9)(1), 2.9);  // y
  EXPECT_EQ(kPpTrajFromPpMatrix.value(1)(2), 2);  // y2 = 2 * y
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2)(2), 4);
  EXPECT_EQ(kPpTrajFromPpMatrix.value(3)(2), 6);
}

}  // namespace
}  // namespace systems
}  // namespace drake
