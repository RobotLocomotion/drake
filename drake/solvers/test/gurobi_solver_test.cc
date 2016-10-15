#include "drake/solvers/gurobi_solver.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace drake {
namespace solvers {
namespace {

void RunGurobiSolver(MathematicalProgram *prog) {
  GurobiSolver gurobi_solver;
  SolutionResult result = SolutionResult::kUnknownError;
  ASSERT_NO_THROW(result = gurobi_solver.Solve(*prog));
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
}

/// Simple test from the Gurobi documentation.
//  min    x^2 + x*y + y^2 + y*z + z^2 + 2 x
//  subj to  x + 2 y + 3 z >= 4
//           x +   y       >= 1
GTEST_TEST(testGurobi, gurobiQPExample1) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3);

  Eigen::MatrixXd Q = 2 * Eigen::Matrix<double, 3, 3>::Identity();
  Q(0, 1) = 1;
  Q(1, 2) = 1;
  Q(1, 0) = 1;
  Q(2, 1) = 1;

  prog.AddBoundingBoxConstraint(
      MatrixXd::Constant(3, 1, 0),
      MatrixXd::Constant(3, 1, std::numeric_limits<double>::infinity()));

  Eigen::VectorXd b(3);
  b << 2.0, 0.0, 0.0;

  prog.AddQuadraticCost(Q, b);

  VectorXd constraint(3);
  constraint << 1, 2, 3;
  prog.AddLinearConstraint(
      constraint.transpose(), drake::Vector1d::Constant(4),
      drake::Vector1d::Constant(std::numeric_limits<double>::infinity()));

  VectorXd constraint2(3);
  constraint2 << 1, 1, 0;
  prog.AddLinearConstraint(
      constraint2.transpose(), drake::Vector1d::Constant(1),
      drake::Vector1d::Constant(std::numeric_limits<double>::infinity()));

  VectorXd expected(3);
  expected << 0, 1, 2.0 / 3.0;

  RunGurobiSolver(&prog);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Closed form (exact) solution test of QP problem.
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
// The test also verifies the quadratic program works when
// matrix Q has off-diagonal terms.
GTEST_TEST(testGurobi, convexQPExample) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(5);
  MatrixXd Q = MatrixXd::Constant(5, 5, 0.0);
  VectorXd Qdiag = VectorXd::Constant(5, 0.0);
  Qdiag << 5.5, 6.5, 6.0, 5.3, 7.5;
  Q = Qdiag.asDiagonal();
  Q(2, 3) = 0.2;

  Eigen::VectorXd b = VectorXd::Constant(5, 0.0);

  b << 3.2, 1.3, 5.6, 9.0, 1.2;

  prog.AddQuadraticCost(Q, b);

  // Exact solution.
  MatrixXd Q_transpose = Q;
  Q_transpose.transposeInPlace();
  MatrixXd Q_symmetric = 0.5 * (Q + Q_transpose);
  VectorXd expected = -Q_symmetric.colPivHouseholderQr().solve(b);
  RunGurobiSolver(&prog);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Closed form (exact) solution test of QP problem.
// Added as multiple QP cost terms
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
// We impose the cost
//   0.5 * x.head<4>()'*Q1 * x.head<4>() + b1'*x.head<4>()
// + 0.5 * x.tail<4>()'*Q2 * x.tail<4>() + b2'*x.tail<4>()
// This is to test that we can add multiple costs for the same variables (in
// this case, the quadratic costs on x(2), x(3) are added for twice).
GTEST_TEST(testGurobi, convexQPMultiCostExample) {
  MathematicalProgram prog;
  const DecisionVariableView x = prog.AddContinuousVariables(6, "x");
  MatrixXd Q1 = MatrixXd::Constant(4, 4, 0.0);
  VectorXd Q1diag = VectorXd::Constant(4, 0.0);
  Q1diag << 5.5, 6.5, 6.0, 7.0;
  Q1 = Q1diag.asDiagonal();
  Q1(1, 2) = 0.1;

  MatrixXd Q2 = MatrixXd::Constant(4, 4, 0.0);
  VectorXd Q2diag = VectorXd::Constant(4, 0.0);
  Q2diag << 7.0, 2.2, 1.1, 1.3;
  Q2 = Q2diag.asDiagonal();
  Q2(0, 2) = -0.02;

  VectorXd b1 = VectorXd::Constant(4, 0.0);
  b1 << 3.1, -1.4, -5.6, 0.6;
  VectorXd b2 = VectorXd::Constant(4, 0.0);
  b2 << 2.3, -5.8, 6.7, 2.3;

  prog.AddQuadraticCost(Q1, b1, {x.head(2), x.segment(2, 2)});
  prog.AddQuadraticCost(Q2, b2, {x.segment(2, 2), x.segment(4, 2)});

  MatrixXd Q = Eigen::MatrixXd::Constant(6, 6, 0.0);
  Q.topLeftCorner(4, 4) = Q1;
  Q.bottomRightCorner(4, 4) += Q2;
  MatrixXd Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Q = 0.5 * (Q + Q_transpose);

  VectorXd b = VectorXd::Constant(6, 0.0);
  b.topRows(4) = b1;
  b.bottomRows(4) += b2;

  // Exact solution.
  VectorXd expected = -Q.ldlt().solve(b);

  RunGurobiSolver(&prog);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Test the simple QP
// min x(0)^2 + x(1)^2 + 2 * x(2)^2
// s.t x(0) +   x(1) = 1
//     x(0) + 2*x(2) = 2
// The optimal solution should be
// x(0) = 4/5, x(1) = 1/5, x(2) = 3/5
GTEST_TEST(testGurobi, addLinearConstraintTest) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3);

  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  Q(2, 2) = 2.0;
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  prog.AddQuadraticCost(Q, b);
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 1), Vector1d::Constant(1), {x(0), x(1)});
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 2), Vector1d::Constant(2), {x(0), x(2)});

  RunGurobiSolver(&prog);
  EXPECT_TRUE(CompareMatrices(Vector3d(0.8, 0.2, 0.6), x.value(), 1e-10, MatrixCompareType::absolute));
}

/**
 * This test is taken from
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_ell_sep.html
 * The goal is to find a hyperplane, that separates two ellipsoids
 * E1 = x1 + R1 * u1, u1' * u1<=1
 * E2 = x2 + R2 * u2, u2' * u2<=1
 * A hyperplane a' * x = b separates these two ellipsoids, if and only if for
 * SOCP p* = min t1 + t2f
 *           s.t t1 >= |R1' * a|
 *               t2 >= |R2' * a|
 *               a'*(x2-x1) = 1
 * the optimal solution p* is no larger than 1. In that case, an approppriate
 * value of b is b = 0.5 * (b1 + b2), where
 * b1 = a' * x1 + |R1' * a|
 * b2 = a' * x2 - |R2' * a|
 * @param x1  the center of ellipsoid 1
 * @param x2  the center of ellipsoid 2
 * @param R1  the shape of ellipsoid 1
 * @param R2  the shape of ellipsoid 2
 */
template<typename DerivedX1, typename DerivedX2, typename DerivedR1, typename DerivedR2>
void testEllipsoidsSeparation(const Eigen::MatrixBase<DerivedX1> &x1,
                              const Eigen::MatrixBase<DerivedX2> &x2,
                              const Eigen::MatrixBase<DerivedR1> &R1,
                              const Eigen::MatrixBase<DerivedR2> &R2) {
  DRAKE_ASSERT(x1.cols() == 1);
  DRAKE_ASSERT(x2.cols() == 1);
  DRAKE_ASSERT(x1.rows() == x2.rows());
  DRAKE_ASSERT(x1.rows() == R1.rows());
  DRAKE_ASSERT(x2.rows() == R2.rows());

  MathematicalProgram prog;
  const int kXdim = x1.rows();
  auto t = prog.AddContinuousVariables(2,"t");
  auto a = prog.AddContinuousVariables(kXdim, "a");
  // R1a = R1' * a
  // R2a = R2' * a
  auto R1a = prog.AddContinuousVariables(R1.cols(), "R1a");
  auto R2a = prog.AddContinuousVariables(R2.cols(), "R2a");
  MatrixXd R1_transpose = R1;
  R1_transpose.transposeInPlace();
  MatrixXd R2_transpose = R2;
  R2_transpose.transposeInPlace();
  MatrixXd A_R1a(R1.cols(), R1.cols() + R1.rows());
  A_R1a.block(0, 0, R1.cols(), R1.cols()) = MatrixXd::Identity(R1.cols(), R1.cols());
  A_R1a.block(0, R1.cols(), R1.cols(), R1.rows()) = -R1_transpose;
  MatrixXd A_R2a(R2.cols(), R2.cols() + R2.rows());
  A_R2a.block(0, 0, R2.cols(), R2.cols()) = MatrixXd::Identity(R2.cols(), R2.cols());
  A_R2a.block(0, R2.cols(), R2.cols(), R2.rows()) = -R2_transpose;
  VectorXd b1 = VectorXd::Zero(R1.cols());
  VectorXd b2 = VectorXd::Zero(R2.cols());
  prog.AddLinearEqualityConstraint(A_R1a, b1, {R1a, a});
  prog.AddLinearEqualityConstraint(A_R2a, b2, {R2a, a});

  // a'*(x2 - x1) = 1
  prog.AddLinearEqualityConstraint((x2 - x1).transpose(), drake::Vector1d(1.0), {a});

  // Add cost
  auto cost = prog.AddLinearCost(Eigen::RowVector2d(1.0, 1.0), {t});

  // Add Lorentz cones
  auto lorentz_cone1 = prog.AddLorentzConeConstraint({t(0), R1a});
  auto lorentz_cone2 = prog.AddLorentzConeConstraint({t(1), R2a});

  RunGurobiSolver(&prog);

  // Check the solution.
  // First check if each constraint is satisfied.
  EXPECT_TRUE(CompareMatrices(R1a.value(), R1_transpose * a.value(), 1e-8, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(R2a.value(), R2_transpose * a.value(), 1e-8, MatrixCompareType::absolute));
  EXPECT_TRUE(std::abs(t.value().coeff(0) - R1a.value().norm()) <= 1e-6);
  EXPECT_TRUE(std::abs(t.value().coeff(1) - R2a.value().norm()) <= 1e-6);
  EXPECT_TRUE(CompareMatrices((x2 - x1).transpose()*a.value(), drake::Vector1d(1.0), 1e-8, MatrixCompareType::absolute));
};
GTEST_TEST(testGurobi, EllipsoidsSeparation) {
  // First test if two balls can be separated
  Vector3d x1 = Vector3d::Zero();
  Vector3d x2 = Vector3d::Zero();
  x2(0) = 2.0;
  Eigen::Matrix3d R1 = 0.5 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R2 = Eigen::Matrix3d::Identity();
  testEllipsoidsSeparation(x1, x2, R1, R2);
}
}  // close namespace
}  // close namespace solvers
}  // close namespace drake
