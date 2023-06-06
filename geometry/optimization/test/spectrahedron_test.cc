#include "drake/geometry/optimization/spectrahedron.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::MathematicalProgram;
using solvers::Solve;

GTEST_TEST(SpectrahedronTest, Empty) {
  Spectrahedron spect;
  EXPECT_EQ(spect.ambient_dimension(), 0);
}

GTEST_TEST(SpectrahedronTest, Attributes) {
  EXPECT_GT(Spectrahedron::supported_attributes().count(
                solvers::ProgramAttribute::kPositiveSemidefiniteConstraint),
            0);
}

GTEST_TEST(SpectrahedronTest, UnsupportedProgram) {
  MathematicalProgram prog;
  symbolic::Variable x = prog.NewContinuousVariables<1>()[0];
  prog.AddConstraint(x * x * x + x >= 0);  // Add a generic constraint.
  DRAKE_EXPECT_THROWS_MESSAGE(Spectrahedron(prog), ".*does not support.*");
}

/*
 * A trivial SDP
 * max X1(0, 1) + X1(1, 2),
 * s.t X1 ∈ ℝ³ˣ³ is psd,
 *     X1(0, 0) + X1(1, 1) + X1(2, 2) = 1,
 *     -2 ≤ X1(0, 1) + X1(1, 2) - 2 * X1(0, 2) ≤ 0,
 *     X1(2, 2) ∈ [-1, 1].
 */
GTEST_TEST(SpectrahedronTest, TrivialSdp1) {
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<3>();
  prog.AddLinearCost(-(X1(0, 1) + X1(1, 2)));
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddLinearEqualityConstraint(X1(0, 0) + X1(1, 1) + X1(2, 2), 1);
  prog.AddLinearConstraint(X1(0, 1) + X1(1, 2) - 2 * X1(0, 2), -2, 0);
  prog.AddBoundingBoxConstraint(-1, 1, X1(2, 2));
  auto result = Solve(prog);
  ASSERT_TRUE(result.is_success());
  const Vector6d x_star = result.GetSolution(prog.decision_variables());
  Vector6d x_bad;
  x_bad << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

  Spectrahedron spect(prog);
  EXPECT_EQ(spect.ambient_dimension(), 3 * (3 + 1) / 2);
  DRAKE_EXPECT_THROWS_MESSAGE(spect.IsBounded(), ".*not implemented yet.*");

  const double kTol{1e-6};
  EXPECT_TRUE(spect.PointInSet(x_star, kTol));
  EXPECT_FALSE(spect.PointInSet(x_bad, kTol));

  MathematicalProgram prog2;
  auto x2 = prog2.NewContinuousVariables<6>("x");
  spect.AddPointInSetConstraints(&prog2, x2);
  EXPECT_EQ(prog2.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog2.linear_equality_constraints().size(), 1);
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->GetDenseA(),
      prog2.linear_equality_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_equality_constraints()[0].evaluator()->lower_bound(),
      prog2.linear_equality_constraints()[0].evaluator()->lower_bound()));
  EXPECT_EQ(prog2.linear_constraints().size(), 1);
  EXPECT_TRUE(
      CompareMatrices(prog.linear_constraints()[0].evaluator()->GetDenseA(),
                      prog2.linear_constraints()[0].evaluator()->GetDenseA()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->lower_bound(),
      prog2.linear_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->upper_bound(),
      prog2.linear_constraints()[0].evaluator()->upper_bound()));
  EXPECT_EQ(prog2.bounding_box_constraints().size(), 1);
  EXPECT_TRUE(CompareMatrices(
      prog.bounding_box_constraints()[0].evaluator()->lower_bound(),
      prog2.bounding_box_constraints()[0].evaluator()->lower_bound()));
  EXPECT_TRUE(CompareMatrices(
      prog.bounding_box_constraints()[0].evaluator()->upper_bound(),
      prog2.bounding_box_constraints()[0].evaluator()->upper_bound()));

  MathematicalProgram prog3;
  auto x3 = prog3.NewContinuousVariables<6>("x");
  auto t3 = prog3.NewContinuousVariables<1>("t");
  spect.AddPointInNonnegativeScalingConstraints(&prog3, x3, t3[0]);
  EXPECT_EQ(prog3.positive_semidefinite_constraints().size(), 1);
  EXPECT_EQ(prog3.linear_equality_constraints().size(), 1);
  MatrixXd A(1, 4);
  A << 1, 1, 1, -1;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_equality_constraints()[0].evaluator()->GetDenseA(), A));
  A.resize(1, 2);
  A << 1, 1;
  EXPECT_EQ(prog3.linear_constraints().size(),
            2 /* from the bounding box constraint */ +
                2 /* from the linear constraint */);
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[0].evaluator()->GetDenseA(), A));
  A << 1, -1;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[1].evaluator()->GetDenseA(), A));
  A.resize(1, 4);
  A << 1, -2, 1, 2;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[2].evaluator()->GetDenseA(), A));
  A << 1, -2, 1, 0;
  EXPECT_TRUE(CompareMatrices(
      prog3.linear_constraints()[3].evaluator()->GetDenseA(), A));
  Vector<double, 7> xt_test;
  xt_test << x_star, 1;
  EXPECT_TRUE(prog3.CheckSatisfied(prog3.GetAllConstraints(), xt_test));
  EXPECT_TRUE(
      prog3.CheckSatisfied(prog3.GetAllConstraints(), VectorXd::Zero(7)));
  xt_test << x_bad, 1;
  EXPECT_FALSE(prog3.CheckSatisfied(prog3.GetAllConstraints(), xt_test));

  DRAKE_EXPECT_THROWS_MESSAGE(
      spect.AddPointInNonnegativeScalingConstraints(
          &prog3, MatrixXd::Identity(6, 6), Vector6d::Zero(), Vector1d::Zero(),
          0, x3, t3),
      ".*not implemented yet.*");
}

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
