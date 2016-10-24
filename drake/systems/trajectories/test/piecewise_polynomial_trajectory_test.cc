#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

#include "gtest/gtest.h"

#include "drake/systems/trajectories/PiecewisePolynomial.h"

namespace drake {
namespace systems {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;
typedef PiecewisePolynomialTrajectory PPTrajType;

GTEST_TEST(piecewisePolynomialTrajectoryTest, testBasicFunctionality) {
  const Polynomiald a = 3;
  const Polynomiald y = Polynomiald("y");
  const Polynomiald y2 = (2 * y);
  const std::vector<Polynomiald> p_vec{a, y, y2};
  const std::vector<double> times{0.0, 3, 7, 9};  // Knot points.
  const PiecewisePolynomialType ppFromVec(p_vec, times);
  const PPTrajType ppTrajFromVec {ppFromVec};
  EXPECT_EQ(ppTrajFromVec.rows(), 1);
  EXPECT_EQ(ppTrajFromVec.cols(), 1);
  // Test first segment 0 <= t < 3, returning the constant a=3.
  EXPECT_EQ(ppTrajFromVec.value(0.0).value(), 3);
  EXPECT_EQ(ppTrajFromVec.value(2).value(), 3);
  EXPECT_EQ(ppTrajFromVec.value(2.9).value(), 3);
  // Test second segment, 3 =< t < 7. Must subtract the knot point 3 when
  // evaluating the variable.
  EXPECT_EQ(ppTrajFromVec.value(3).value(), y.EvaluateUnivariate(3 - 3));
  EXPECT_EQ(ppTrajFromVec.value(4)(0), y.EvaluateUnivariate(4 - 3));
  // Test third segment, 7 =< t < 9. Must subtract the knot point 7 when
  // evaluating the variable.
  EXPECT_EQ(ppTrajFromVec.value(7)(0), y2.EvaluateUnivariate(7 - 7));
  EXPECT_EQ(ppTrajFromVec.value(8)(0), y2.EvaluateUnivariate(8 - 7));
  EXPECT_EQ(ppTrajFromVec.value(8.9)(0), y2.EvaluateUnivariate(8.9 - 7));

  // Enter the matrix
  std::vector<PiecewisePolynomialType::PolynomialMatrix> ppMatrix(1);
  ppMatrix[0].resize(3, 1);
  ppMatrix[0](0) = a;
  ppMatrix[0](1) = y;
  ppMatrix[0](2) = y2;

  const PiecewisePolynomialType ppFromMatrix(ppMatrix, {0.0, 3});
  const PPTrajType ppTrajFromMatrix {ppFromMatrix};
  EXPECT_EQ(ppTrajFromMatrix.rows(), 3);
  EXPECT_EQ(ppTrajFromMatrix.cols(), 1);
  EXPECT_EQ(ppTrajFromMatrix.value(2.9)(0), 3);
  EXPECT_EQ(ppTrajFromMatrix.value(2.9)(1), 2.9);  // y
  EXPECT_EQ(ppTrajFromMatrix.value(1)(2), 2);  // y2 = 2 * y
  EXPECT_EQ(ppTrajFromMatrix.value(2)(2), 4);
  EXPECT_EQ(ppTrajFromMatrix.value(3)(2), 6);
}

}  // namespace
}  // namespace systems
}  // namespace drake
