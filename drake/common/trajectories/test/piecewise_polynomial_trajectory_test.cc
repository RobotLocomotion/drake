#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

#include <Eigen/Core>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
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
  const Polynomiald yyyy = (y * y * y * y);

  // Construct a vector of polynomials.
  const std::vector<Polynomiald> p_vec {a, y, y2};
  const std::vector<double> times {1.0, 3, 7, 9};  // Knot points.

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
  EXPECT_EQ(kPpTrajFromVec.get_start_time(), times[0]);
  EXPECT_EQ(kPpTrajFromVec.get_end_time(), times.back());

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

  // Test derivative()
  EXPECT_TRUE(CompareMatrices(kPpFromVec.derivative(1).value(1),
                              kPpTrajFromVec.derivative(1)->value(1), 1e-10,
                              MatrixCompareType::absolute));

  // Test Clone().
  std::unique_ptr<Trajectory> clone = kPpTrajFromVec.Clone();
  EXPECT_TRUE(CompareMatrices(kPpTrajFromVec.value(1),
                              clone->value(1), 1e-10,
                              MatrixCompareType::absolute));

  // Test: construct a PiecewisePolynomialTrajectory from a PP matrix.

  // Construct a matrix of polynomials.
  std::vector<PiecewisePolynomialType::PolynomialMatrix> kPpMatrix(1);
  kPpMatrix[0].resize(4, 1);
  kPpMatrix[0](0) = a;
  kPpMatrix[0](1) = y;
  kPpMatrix[0](2) = y2;
  kPpMatrix[0](3) = yyyy;

  // Create a PiecewisePolynomial from a matrix.
  const PiecewisePolynomialType kPpFromMatrix(kPpMatrix, {0.0, 3});

  // Create a PiecewisePolynomialTrajectory from a PP made from a matrix.
  const PiecewisePolynomialTrajectory kPpTrajFromPpMatrix {kPpFromMatrix};
  EXPECT_EQ(kPpTrajFromPpMatrix.rows(), 4);
  EXPECT_EQ(kPpTrajFromPpMatrix.cols(), 1);
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2.9)(0), 3);  // @0: a = 3.
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2.9)(1), 2.9);  // @1: y = t.
  EXPECT_EQ(kPpTrajFromPpMatrix.value(1)(2), 2);  // @2: y2 = 2 * y = 2 * t.
  EXPECT_EQ(kPpTrajFromPpMatrix.value(2)(2), 4);
  EXPECT_EQ(kPpTrajFromPpMatrix.value(3)(2), 6);

  // Test derivative()
  EXPECT_TRUE(CompareMatrices(kPpFromMatrix.derivative(0).value(1.5),
                              kPpTrajFromPpMatrix.derivative(0)->value(1.5),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(kPpFromMatrix.derivative(1).value(1.5),
                              kPpTrajFromPpMatrix.derivative(1)->value(1.5),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(kPpFromMatrix.derivative(2).value(1.5),
                              kPpTrajFromPpMatrix.derivative(2)->value(1.5),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(kPpFromMatrix.derivative(3).value(1.5),
                              kPpTrajFromPpMatrix.derivative(3)->value(1.5),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(kPpFromMatrix.derivative(4).value(1.5),
                              kPpTrajFromPpMatrix.derivative(4)->value(1.5),
                              1e-10, MatrixCompareType::absolute));
  // @0: f(t) = a = 3; f'(t) = 0.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative()->value(2.5)(0), 0);

  // @1: f(t) = y; f'(t) = 1.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative()->value(2.5)(1), 1);
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(2)->value(2.5)(1), 0);

  // @2: f(t) = y2 = 2 * y; f'(t) = 2 * 1 = 2.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative()->value(2.5)(2), 2);
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(2)->value(2.5)(2), 0);

  // @3: f(t) = yyyy = y^4.
  // f'(t) = 4 * y^3 = 4 * t^3 = 4 * 2^3 = 32.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(1)->value(2)(3), 32);
  // f''(t) = 12 * y^2.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(2)->value(2)(3), 48);
  // f'''(t) = 24 * y.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(3)->value(2)(3), 48);
  // f''''(t) = 24.
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(4)->value(200)(3), 24);
  EXPECT_EQ(kPpTrajFromPpMatrix.derivative(5)->value(200)(3), 0);
}

GTEST_TEST(piecewisePolynomialTrajectoryTest, PPAccessor) {
  std::vector<Eigen::MatrixXd> points;
  std::vector<double> times;
  constexpr int num_points = 4;
  for (int i = 0; i < num_points; ++i) {
    times.push_back(static_cast<double>(i));
    points.push_back(Eigen::VectorXd::Random(3, 1));
  }
  const auto poly = PiecewisePolynomial<double>::FirstOrderHold(times, points);
  const PiecewisePolynomialTrajectory poly_traj(poly);
  const auto& poly_traj_poly = poly_traj.get_piecewise_polynomial();
  for (int i = 0; i < num_points - 1; ++i) {
    EXPECT_EQ(poly.getPolynomialMatrix(i),
              poly_traj_poly.getPolynomialMatrix(i));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
