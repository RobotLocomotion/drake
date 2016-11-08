#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

#include "gtest/gtest.h"

#include "drake/systems/trajectories/PiecewisePolynomial.h"

namespace drake {
namespace systems {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

GTEST_TEST(piecewisePolynomialTrajectoryTest, testBasicFunctionality) {
  // Setup testing data
  const Polynomiald a = 3;
  const Polynomiald y = Polynomiald("y");
  const Polynomiald y2 = (2 * y);

  // Construct a vector of polynomials.
  const std::vector<Polynomiald> p_vec {a, y, y2};
  const std::vector<double> times {0.0, 3, 7, 9};  // Knot points.

  // Test constructing from a pp vector.

  // Create a PiecewisePolynomial from a vector.
  const PiecewisePolynomialType ppFromVec(p_vec, times);

  // Create a PiecewisePolynomialTrajectory from a PP made from a vector.
  const PiecewisePolynomialTrajectory ppTrajFromVec {ppFromVec};
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

  // Test constructing from a pp matrix.

  // Construct a matrix of polynomials.
  std::vector<PiecewisePolynomialType::PolynomialMatrix> ppMatrix(1);
  ppMatrix[0].resize(3, 1);
  ppMatrix[0](0) = a;
  ppMatrix[0](1) = y;
  ppMatrix[0](2) = y2;

  // Create a PiecewisePolynomial from a matrix.
  const PiecewisePolynomialType ppFromMatrix(ppMatrix, {0.0, 3});

  // Create a PiecewisePolynomialTrajectory from a PP made from a matrix.
  const PiecewisePolynomialTrajectory pp_traj_from_pp_matrix {ppFromMatrix};
  EXPECT_EQ(pp_traj_from_pp_matrix.rows(), 3);
  EXPECT_EQ(pp_traj_from_pp_matrix.cols(), 1);
  EXPECT_EQ(pp_traj_from_pp_matrix.value(2.9)(0), 3);
  EXPECT_EQ(pp_traj_from_pp_matrix.value(2.9)(1), 2.9);  // y
  EXPECT_EQ(pp_traj_from_pp_matrix.value(1)(2), 2);  // y2 = 2 * y
  EXPECT_EQ(pp_traj_from_pp_matrix.value(2)(2), 4);
  EXPECT_EQ(pp_traj_from_pp_matrix.value(3)(2), 6);

  // Test constructing from a matrix.

  const int kNumJoints {3};
  // Each column represents a particular time, and the rows of that
  // column contain values for each joint coordinate.
  MatrixXd traj_matrix(kNumJoints, times.size());
  traj_matrix <<
      9.9, 2, 3, 4.1,
      6, 5, 4.5, 5.1,
      7, 6, 4, 2.1;

  // Create a PiecewisePolynomialTrajectory from a matrix.
  const PiecewisePolynomialTrajectory pp_traj_from_matrix {traj_matrix, times};
  EXPECT_EQ(pp_traj_from_matrix.rows(), kNumJoints);
  EXPECT_EQ(pp_traj_from_matrix.cols(), 1);
  // There is interpolation between points in the input matrix (traj_matrix),
  // but in this test we test just at the knot points (times).
  // The output from value() is a vector of length = kNumJoints.
  EXPECT_EQ(pp_traj_from_matrix.value(times[0])(0), traj_matrix(0,0));
  EXPECT_EQ(pp_traj_from_matrix.value(times[0])(1), traj_matrix(1,0));
  EXPECT_EQ(pp_traj_from_matrix.value(times[0])(2), traj_matrix(2,0));

  // Don't bother testing every knot point; just boundaries.
  EXPECT_EQ(pp_traj_from_matrix.value(times[1])(2), traj_matrix(2,1));
  EXPECT_EQ(pp_traj_from_matrix.value(times[2])(2), traj_matrix(2,2));

  EXPECT_EQ(pp_traj_from_matrix.value(times[3])(0), traj_matrix(0,3));
  EXPECT_EQ(pp_traj_from_matrix.value(times[3])(1), traj_matrix(1,3));
  EXPECT_EQ(pp_traj_from_matrix.value(times[3])(2), traj_matrix(2,3));
}

}  // namespace
}  // namespace systems
}  // namespace drake
