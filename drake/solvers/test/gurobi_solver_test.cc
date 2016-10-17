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
  MatrixXd Q_symmetric = 0.5 * (Q + Q.transpose());
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

  // Now check if the solution is meaning, that it really finds a separating
  // hyperplane.
  // The separating hyperplane exists if and only if p* <= 1
  double p_star = t.value().coeff(0) + t.value().coeff(1);
  bool is_separated = p_star <= 1.0;
  double t1 = t.value().coeff(0);
  double t2 = t.value().coeff(1);
  if(is_separated) {
    // Then the hyperplane a' * x = 0.5 * (a'*x1 + t1 + a'*x2 - t2)
    double b1 = a.value().transpose() * x1 + t1;
    double b2 = a.value().transpose() * x2 - t2;
    double b = 0.5 * (b1 + b2);
    // Verify that b - a'*x1 >= |R1' * a|
    //             a'*x2 - b >= |R2' * a|
    EXPECT_TRUE(b - a.value().transpose() * x1 >= (R1_transpose * a.value()).norm());
    EXPECT_TRUE(a.value().transpose() * x2 - b >= (R2_transpose * a.value()).norm());
  }
  else {
    // Now solve another SOCP to find a point y in the intersecting region
    // y = x1 + R1*u1
    // y = x2 + R2*u2
    // 1 >= |u1|
    // 1 >= |u2|
    MathematicalProgram prog_intersect;
    auto u1 = prog_intersect.AddContinuousVariables(R1.cols(),"u1");
    auto u2 = prog_intersect.AddContinuousVariables(R2.cols(),"u2");
    auto y = prog_intersect.AddContinuousVariables(kXdim, "y");

    auto slack = prog_intersect.AddContinuousVariables(1, "slack");
    prog_intersect.AddBoundingBoxConstraint(drake::Vector1d(1), drake::Vector1d(1), {slack});

    prog_intersect.AddLorentzConeConstraint({slack, u1});
    prog_intersect.AddLorentzConeConstraint({slack, u2});

    // Add constraint y = x1 + R1*u1
    //                y = x2 + R2*u2
    MatrixXd A1(y.size(), y.size() + R1.cols());
    A1.block(0, 0, y.size(), y.size()) = MatrixXd::Identity(y.size(), y.size());
    A1.block(0, y.size(), y.size(), R1.cols()) = -R1;
    MatrixXd A2(y.size(), y.size() + R2.cols());
    A2.block(0, 0, y.size(), y.size()) = MatrixXd::Identity(y.size(), y.size());
    A2.block(0, y.size(), y.size(), R2.cols()) = -R2;
    prog_intersect.AddLinearEqualityConstraint(A1, x1, {y, u1});
    prog_intersect.AddLinearEqualityConstraint(A2, x2, {y, u2});

    RunGurobiSolver(&prog_intersect);

    // Check if the constraints are satisfied
    EXPECT_TRUE(u1.value().norm() <= 1);
    EXPECT_TRUE(u2.value().norm() <= 1);
    EXPECT_TRUE(CompareMatrices(y.value(), x1 + R1 * u1.value(), 1e-8, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y.value(), x2 + R2 * u2.value(), 1e-8, MatrixCompareType::absolute));
  }
};
GTEST_TEST(testGurobi, EllipsoidsSeparation) {
  // First test if two balls can be separated
  VectorXd x1 = Vector3d::Zero();
  VectorXd x2 = Vector3d::Zero();
  x2(0) = 2.0;
  Eigen::MatrixXd R1 = 0.5 * Eigen::Matrix3d::Identity();
  Eigen::MatrixXd R2 = Eigen::Matrix3d::Identity();
  testEllipsoidsSeparation(x1, x2, R1, R2);

  // Test if two intersecting balls
  x1 = Vector3d::Zero();
  x2 = Vector3d::Zero();
  x2(0) = 1.0;
  R1 = Eigen::Matrix3d::Identity();
  R2 = Eigen::Matrix3d::Identity();
  testEllipsoidsSeparation(x1, x2, R1, R2);

  // Test two ellipsoids
  x1 = Eigen::Vector2d(1.0, 0.2);
  x2 = Eigen::Vector2d(0.5, 0.4);
  R1 = Eigen::Matrix2d::Zero();
  R1 << 0.1, 0.6,
        0.2, 1.3;
  R2 = Eigen::Matrix2d::Zero();
  R2 << -0.4, 1.5,
        1.7, 0.3;
  testEllipsoidsSeparation(x1, x2, R1, R2);

  // Test another two ellipsoids
  x1 = Eigen::Vector3d(1.0, 0.2, 0.8);
  x2 = Eigen::Vector3d(3.0, -1.5, 1.9);
  R1 = Eigen::Matrix3d::Zero();
  R1 << 0.2, 0.4, 0.2,
        -0.2, -0.1, 0.3,
        0.2, 0.1, 0.1;
  R2 = Eigen::Matrix<double, 3, 2>::Zero();
  R2 << 0.1, 0.2,
        -0.1, 0.01,
        -0.2, 0.1;
  testEllipsoidsSeparation(x1, x2, R1, R2);
}

/**
 * This example is taken from
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_qp_as_socp.html
 * For a quadratic program
 * 0.5 * x' * Q * x + c' * x
 * s.t A * x <= b
 * It can be casted as an SOCP, as follows
 * By introducing a new variable w = Q^{1/2}*x and y, z
 * The equivalent SOCP is
 * min c'x + y
 * s.t y * z >= w' * w
 *     z = 2
 *     w = Q^{1/2} * x
 *     b_lb <= A * x <= b_ub
 * @param Q A positive definite matrix
 * @param c A column vector
 * @param A A matrix
 * @param b_lb A column vector
 * @param b_ub A column vector
 * @return The optimal solution x*.
 */
template<typename DerivedQ, typename DerivedC, typename DerivedA, typename DerivedBlower, typename DerivedBupper>
VectorXd SolveQPasSOCP(const Eigen::MatrixBase<DerivedQ> &Q,
                       const Eigen::MatrixBase<DerivedC> &c,
                       const Eigen::MatrixBase<DerivedA> &A,
                       const Eigen::MatrixBase<DerivedBlower> &b_lb,
                       const Eigen::MatrixBase<DerivedBupper> &b_ub) {
  DRAKE_ASSERT(Q.rows() == Q.cols());
  MatrixXd Q_symmetric = 0.5 * (Q + Q.transpose());
  const int kXdim = Q.rows();
  DRAKE_ASSERT(c.rows() == kXdim);
  DRAKE_ASSERT(c.cols() == 1);
  DRAKE_ASSERT(A.cols() == kXdim);
  DRAKE_ASSERT(A.rows() == b_lb.rows());
  DRAKE_ASSERT(b_lb.cols() == 1);
  DRAKE_ASSERT(A.rows() == b_ub.rows());
  DRAKE_ASSERT(b_ub.cols() == 1);

  MathematicalProgram prog_socp;

  auto x_socp = prog_socp.AddContinuousVariables(kXdim, "x");
  auto y = prog_socp.AddContinuousVariables(1, "y");
  auto z = prog_socp.AddContinuousVariables(1, "z");
  auto w = prog_socp.AddContinuousVariables(kXdim, "w");

  prog_socp.AddBoundingBoxConstraint(drake::Vector1d(2.0), drake::Vector1d(2.0), {z});
  prog_socp.AddRotatedLorentzConeConstraint({y, z, w});

  Eigen::LLT<MatrixXd, Eigen::Upper> lltOfQ(Q_symmetric);
  MatrixXd Q_sqrt = lltOfQ.matrixU();
  MatrixXd A_w(kXdim, 2*kXdim);
  A_w << MatrixXd::Identity(kXdim, kXdim), -Q_sqrt;
  prog_socp.AddLinearEqualityConstraint(A_w, VectorXd::Zero(kXdim), {w, x_socp});

  prog_socp.AddLinearConstraint(A, b_lb, b_ub, {x_socp});

  std::shared_ptr<LinearConstraint> cost_socp1(new LinearConstraint(c.transpose(), drake::Vector1d(-std::numeric_limits<double>::infinity()), drake::Vector1d(std::numeric_limits<double>::infinity())));
  prog_socp.AddCost(cost_socp1, {x_socp});
  prog_socp.AddLinearCost(drake::Vector1d(1.0), {y});
  RunGurobiSolver(&prog_socp);
  double objective_value_socp = c.transpose() * x_socp.value() + y.value().coeff(0);

  // Check the solution
  EXPECT_TRUE(std::abs(2*y.value().coeff(0) - w.value().squaredNorm()) <= 1E-6);
  EXPECT_TRUE(CompareMatrices(w.value(), Q_sqrt * x_socp.value(), 1e-6, MatrixCompareType::absolute));
  EXPECT_TRUE(y.value().coeff(0) >= 0);
  EXPECT_TRUE(CompareMatrices(w.value(), Q_sqrt*x_socp.value(), 1E-6, MatrixCompareType::absolute));

  // Now solve the problem as a QP.
  MathematicalProgram prog_qp;
  auto x_qp = prog_qp.AddContinuousVariables(kXdim, "x");
  prog_qp.AddQuadraticCost(Q, c, {x_qp});
  prog_qp.AddLinearConstraint(A, b_lb, b_ub, {x_qp});
  RunGurobiSolver(&prog_qp);
  double objective_value_qp = 0.5 * (x_qp.value().transpose() * Q * x_qp.value()).coeff(0, 0) + c.transpose() * x_qp.value();

  // TODO(hongkai.dai@tri.global):tighten the tolerance. socp does not really converge to true optimal yet.
  EXPECT_TRUE(CompareMatrices(x_qp.value(), x_socp.value(), 1e-4, MatrixCompareType::absolute));
  EXPECT_TRUE(std::abs(objective_value_qp - objective_value_socp) < 1E-6);
  return x_socp.value();
};
GTEST_TEST(testGurobi, RotatedLorentzConeTest) {
  // Solve an un-constrained QP
  MatrixXd Q = Eigen::Matrix2d::Identity();
  VectorXd c = Eigen::Vector2d::Ones();
  MatrixXd A = Eigen::RowVector2d(0, 0);
  VectorXd b_lb = VectorXd::Constant(1, -std::numeric_limits<double>::infinity());
  VectorXd b_ub = VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  SolveQPasSOCP(Q, c, A, b_lb, b_ub);

  // Solve a constrained QP
  Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = 1.0;
  Q(1, 1) = 1.3;
  Q(2, 2) = 2.0;
  Q(1, 2) = 0.01;
  Q(0, 1) = -0.2;
  c = Vector3d::Zero();
  c(0) = -1.0;
  c(1) = -2.0;
  c(2) = 1.2;

  A = Eigen::Matrix<double, 2, 3>::Zero();
  A << 1, 0, 2,
       0, 1, 3;
  b_lb = Eigen::Vector2d::Zero();
  b_lb(0) = -1;
  b_lb(1) = -2;
  b_ub = Eigen::Vector2d::Zero();
  b_ub(0) = 2;
  b_ub(1) = 4;
  SolveQPasSOCP(Q, c, A, b_lb, b_ub);
}
}  // close namespace
}  // close namespace solvers
}  // close namespace drake
