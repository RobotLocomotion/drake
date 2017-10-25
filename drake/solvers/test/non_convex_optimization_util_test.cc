#include "drake/solvers/non_convex_optimization_util.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
GTEST_TEST(DecomposeNonConvexQuadraticForm, Test0) {
  // Decomposes a PSD matrix Q. This should yield Q1 = Q and Q2 = 0.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Q, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test1) {
  // Decomposes a negative definite matrix Q. This should yield Q1 = 0, and Q2 =
  // -Q.
  Eigen::Matrix3d Q1, Q2;
  Eigen::Matrix3d Q = -Eigen::Matrix3d::Identity();
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(CompareMatrices(Q1, Eigen::Matrix3d::Zero(), 1E-5,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(Q2, -Q, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test2) {
  // Decomposes an indefinite matrix Q, [0, 1; 1 0].
  // This should yield Q1 = [1 1; 1 1]/2, Q2 = [1 -1; -1 1]/2.
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 0, 1, 1, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.0 / 2, 1.0 / 2, 1.0 / 2, 1.0 / 2;
  Q2_expected << 1.0 / 2, -1.0 / 2, -1.0 / 2, 1.0 / 2;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));

  // Decomposes another indefinite matrix Q, [0, 2; 0, 0], this matrix has the
  // same quadratic form as [0, 1; 1 0], so it should give the same Q1 and Q2.
  Q << 0, 2, 0, 0;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}

GTEST_TEST(DecomposeNonConvexQuadraticForm, Test3) {
  // Decomposes an indefinite matrix Q = [1 3; 1, 1].
  // This should yield Q1 = [1.5 1.5; 1.5 1.5], Q2 = [0.5 -0.5; -0.5 0.5]
  Eigen::Matrix2d Q1, Q2;
  Eigen::Matrix2d Q;
  Q << 1, 3, 1, 1;
  std::tie(Q1, Q2) = DecomposeNonConvexQuadraticForm(Q);
  Eigen::Matrix2d Q1_expected, Q2_expected;
  Q1_expected << 1.5, 1.5, 1.5, 1.5;
  Q2_expected << 0.5, -0.5, -0.5, 0.5;
  EXPECT_TRUE(
      CompareMatrices(Q1, Q1_expected, 1E-5, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(Q2, Q2_expected, 1E-5, MatrixCompareType::absolute));
}

void CheckRelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
    const Eigen::Ref<const Eigen::MatrixXd>& Q1,
    const Eigen::Ref<const Eigen::MatrixXd>& Q2,
    const Eigen::Ref<const Eigen::VectorXd>& p, double upper_bound,
    const Eigen::Ref<const Eigen::VectorXd>& linearization_point,
    double trust_region_gap) {
  const auto constraint =
      RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
          Q1, Q2, p, upper_bound, linearization_point, trust_region_gap);
  VectorXDecisionVariable x(Q1.rows());
  for (int i = 0; i < Q1.rows(); ++i) {
    x(i) = symbolic::Variable("x" + std::to_string(i));
  }
  const VectorX<symbolic::Expression> y = constraint->A() * x + constraint->b();
  EXPECT_PRED2(
      symbolic::test::ExprEqual, (y(0) * y(1)).Expand(),
      (upper_bound + trust_region_gap + 2 * linearization_point.dot(Q2 * x) -
       linearization_point.dot(Q2 * linearization_point) - p.dot(x))
          .Expand());
  EXPECT_PRED2(symbolic::test::ExprEqual,
               y.tail(Q1.rows()).squaredNorm().Expand(),
               (x.dot(Q1 * x)).Expand());
}

GTEST_TEST(TestRelaxNonConvexQuadraticInequalityConstraintInTrustRegion,
           Test0) {
  Eigen::Matrix2d Q1 = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.2, 0.2, 1;
  Eigen::Vector2d p(1, 2);
  double upper_bound{0.1};
  Eigen::Vector2d linearization_point(1, 2);
  double trust_region_gap{1};
  CheckRelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
      Q1, Q2, p, upper_bound, linearization_point, trust_region_gap);
}

GTEST_TEST(TestRelaxNonConvexQuadraticInequalityConstraintInTrustRegion,
           Test1) {
  Eigen::Matrix2d Q1;
  Q1 << 1, 0.2, 0.2, 2;
  Eigen::Matrix2d Q2;
  Q2 << 1, 0.3, 0.3, 4;
  Eigen::Vector2d p(3, 2);
  double upper_bound{-0.1};
  Eigen::Vector2d linearization_point(-1, 2);
  double trust_region_gap{1};
  CheckRelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
      Q1, Q2, p, upper_bound, linearization_point, trust_region_gap);
}

GTEST_TEST(TestRelaxNonConvexQuadraticInequalityConstraintInTrustRegion,
           TestRuntimeError) {
  Eigen::Matrix2d non_positive_Q;
  non_positive_Q << 1, 1.5, 1.5, 1;
  Eigen::Matrix2d positive_Q;
  positive_Q << 1, 0.2, 0.2, 1;
  EXPECT_THROW(RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
                   non_positive_Q, positive_Q, Eigen::Vector2d(1, 0), 0.1,
                   Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  EXPECT_THROW(RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
                   positive_Q, non_positive_Q, Eigen::Vector2d(1, 0), 0.1,
                   Eigen::Vector2d(1, 2), 1),
               std::runtime_error);
  EXPECT_THROW(RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(
                   positive_Q, positive_Q, Eigen::Vector2d(1, 0), 0.1,
                   Eigen::Vector2d(1, 2), -0.1),
               std::runtime_error);
}

GTEST_TEST(TestRelaxNonConvexQuadraticInequalityConstraintInTrustRegion, SolveProblem0) {
  // For a feasibility problem
  // find x, s.t x(0)² - x(1)² <= 2
  // We linearize it about x = (2, 1). If we set the violation to 0.5, within
  // the trust region 2x(1) - 1 + 0.5 >= x(1)², there should be a solution
  // that violates the constraint by at most 0.5
  Eigen::Matrix2d Q1, Q2;
  // We add 1E-10 here to make sure Q1 and Q2 are positive definite.
  Q1 << 1 + 1E-10, 0, 0, 1E-10;
  Q2 << 1E-10, 0, 0, 1 + 1E-10;
  auto constraint = RelaxNonConvexQuadraticInequalityConstraintInTrustRegion(Q1, Q2, Eigen::Vector2d::Zero(), 2, Eigen::Vector2d(2, 1), 0.5);
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddConstraint(constraint, x);
  auto result = prog.Solve();
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  auto x_sol = prog.GetSolution(x);
  EXPECT_LE(std::pow(x_sol(0), 2) - std::pow(x_sol(1), 2), 2 + 0.5);
  EXPECT_GE(2 * x_sol(1) - 1 + 0.5, std::pow(x_sol(1), 2));
}
}  // namespace
}  // namespace solvers
}  // namespace drake
