#include <iostream>

#include <list>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

namespace drake {
namespace solvers {
namespace test {
namespace {
/////////////////////////
///// Linear Program ////
/////////////////////////

// Test a simple linear programming problem with zero cost, i.e. a feasibility
// problem
//    0 <= x0 + 2x1 + 3x2 <= 10
// -inf <=       x1 - 2x2 <= 3
//   -1 <= 0x0+ 0x1 + 0x2 <= 0
//           x1 >= 1
void TestLinearProgramFeasibility(
    const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3, "x");
  Eigen::Matrix<double, 3, 3> A;
  A << 1, 2, 3, 0, 1, -2, 0, 0, 0;
  Eigen::Vector3d b_lb(0, -std::numeric_limits<double>::infinity(), -1);
  Eigen::Vector3d b_ub(10, 3, 0);
  prog.AddLinearConstraint(A, b_lb, b_ub);
  prog.AddBoundingBoxConstraint(1.0, std::numeric_limits<double>::infinity(),
                                x(1));

  RunSolver(&prog, solver);

  Eigen::Vector3d A_times_x = A * x.value();
  EXPECT_GE(A_times_x(0), 0 - 1e-10);
  EXPECT_LE(A_times_x(0), 10 + 1e-10);
  EXPECT_LE(A_times_x(1), 3 + 1E-10);
  EXPECT_LE(A_times_x(2), 0 + 1E-10);
  EXPECT_GE(A_times_x(2), 0 - 1E-10);
  EXPECT_GE(x.value().coeff(1), 1 - 1E-10);
}

// Adapt from the linear programming example
// http://cvxopt.org/examples/tutorial/lp.html
// Solve the following linear program
// min     2x0 + x1
// s.t  -inf <= -x0 + x1 <= 1
//         2 <= x0 + x1  <=inf
//      -inf <= x0 - 2x1 <= 4
//      x1 >= 2
//      x0 >= 0
// The optimal solution is x0 = 1, x1 = 2
void TestLinearProgram0(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");

  prog.AddLinearCost(Eigen::RowVector2d(2.0, 1.0));
  Eigen::Matrix<double, 3, 2> A;
  A << -1, 1, 1, 1, 1, -2;
  Eigen::Vector3d b_lb(-std::numeric_limits<double>::infinity(), 2.0,
                       -std::numeric_limits<double>::infinity());
  Eigen::Vector3d b_ub(1.0, std::numeric_limits<double>::infinity(), 4.0);
  prog.AddLinearConstraint(A, b_lb, b_ub);
  prog.AddBoundingBoxConstraint(
      Eigen::Vector2d(0.0, 2.0),
      Eigen::Vector2d(std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity()),
      {x(0), x(1)});

  RunSolver(&prog, solver);

  Eigen::Vector2d x_expected(1, 2);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1E-10,
                              MatrixCompareType::absolute));
}

// Test a simple linear programming problem with only bounding box constraint
// on x.
// min x0 - 2*x1
//     0 <= x0 <= 2
//    -1 <= x1 <= 4
// The optimal solution is (0, 4)
void TestLinearProgram1(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");
  prog.AddLinearCost(Eigen::RowVector2d(1.0, -2.0));
  prog.AddBoundingBoxConstraint(Eigen::Vector2d(0, -1), Eigen::Vector2d(2, 4));

  RunSolver(&prog, solver);

  EXPECT_TRUE(x.value().isApprox(Eigen::Vector2d(0, 4)));
}

// Test a simple linear programming problem
// Adapt from http://docs.mosek.com/7.1/capi/Linear_optimization.html
// min -3x0 - x1 - 5x2 - x3
// s.t     3x0 +  x1 + 2x2        = 30
//   15 <= 2x0 +  x1 + 3x2 +  x3 <= inf
//  -inf<=       2x1       + 3x3 <= 25
// -inf <=  x0 + 2x1       + x3  <= inf
// -100 <=  x0       + 2x2       <= 40
//           0 <= x0 <= inf
//           0 <= x1 <= 10
//           0 <= x2 <= inf
//           0 <= x3 <= inf
// The optimal solution is at (0, 0, 15, 25/3)
void TestLinearProgram2(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(4, "x");
  // We deliberately break the cost to c1' * [x0;x1;x2] + c2'*[x2;x3] here
  // to test adding multiple costs.
  Eigen::RowVector3d c1(-3, -1, -4);
  Eigen::RowVector2d c2(-1, -1);

  prog.AddLinearCost(c1, {x(0), x(1), x(2)});
  prog.AddLinearCost(c2, {x(2), x(3)});

  Eigen::RowVector3d a1(3, 1, 2);
  prog.AddLinearEqualityConstraint(a1, 30, {x(0), x(1), x(2)});

  Eigen::Matrix<double, 4, 4> A;
  A << 2, 1, 3, 1, 0, 2, 0, 3, 1, 2, 0, 1, 1, 0, 2, 0;
  Eigen::Vector4d b_lb(15, -std::numeric_limits<double>::infinity(),
                       -std::numeric_limits<double>::infinity(), -100);
  Eigen::Vector4d b_ub(std::numeric_limits<double>::infinity(), 25,
                       std::numeric_limits<double>::infinity(), 40);
  prog.AddLinearConstraint(A, b_lb, b_ub);

  prog.AddBoundingBoxConstraint(0, 10, x(1));
  prog.AddBoundingBoxConstraint(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
      {x(0), x(2), x(3)});

  RunSolver(&prog, solver);

  Eigen::Vector4d x_expected(0, 0, 15, 25.0 / 3.0);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-10,
                              MatrixCompareType::absolute));
}

/////////////////////////////////////////
/////////// Quadratic Program ///////////
/////////////////////////////////////////

/*
 * Test a simple Quadratic Program.
 * The example is taken from
 * http://cvxopt.org/examples/tutorial/qp.html
 * min 2x1^2 + x2^2 + x1x2 + x1 + x2
 * s.t x1 >= 0
 *     x2 >= 0
 *     x1 + x2 = 1
 */
void TestQuadraticProgram0(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");

  // Deliberately add the cost as the sum of a quadratic cost
  // 2x1^2 + x2^2 + x1x2 + x1
  // add a linear cost
  // x2
  Eigen::Matrix2d Q;
  // The quadratic cost is 0.5*x'*Q*x+b'*x.
  Q << 4, 2, 0, 2;
  Eigen::Vector2d b(1, 0);
  prog.AddQuadraticCost(Q, b, {x});
  prog.AddLinearCost(drake::Vector1d(1.0), {x(1)});

  // Deliberately add two bounding box constraints
  // (x1, x2) >= (0, 0)
  // and
  // x2 >= -1
  // to test multiple bounding box constraints.
  prog.AddBoundingBoxConstraint(
      Eigen::Vector2d(0, 0),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()), {x});
  prog.AddBoundingBoxConstraint(-1, std::numeric_limits<double>::infinity(),
                                x(1));

  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 1), 1);

  RunSolver(&prog, solver);

  Eigen::Vector2d x_expected(0.25, 0.75);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1E-8,
                              MatrixCompareType::absolute));
}

/// Adapt from the simple test on the Gurobi documentation.
//  min    x^2 + x*y + y^2 + y*z + z^2 + 2 x
//  subj to 4 <=   x + 2 y + 3 z <= inf
//       -inf <=  -x -   y       <= -1
//        -20 <=         y + 2 z <= 100
//       -inf <=   x +   y + 2 z <= inf
//               3 x +   y + 3 z  = 3
//                 x, y, z >= 0
//   The optimal solution is (0, 1, 2/3)
void TestQuadraticProgram1(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3);

  Eigen::MatrixXd Q = 2 * Eigen::Matrix<double, 3, 3>::Identity();
  Q(0, 1) = 1;
  Q(1, 2) = 1;
  Q(1, 0) = 1;
  Q(2, 1) = 1;
  Eigen::VectorXd b(3);
  b << 2.0, 0.0, 0.0;
  prog.AddQuadraticCost(Q, b);

  prog.AddBoundingBoxConstraint(
      Eigen::MatrixXd::Zero(3, 1),
      Eigen::MatrixXd::Constant(3, 1, std::numeric_limits<double>::infinity()));

  // This test handles the case that in one LinearConstraint,
  // some rows have active upper bounds;
  // some rows have active lower bounds;
  // some rows have both bounds;
  // some rows have none.
  Eigen::Matrix<double, 4, 3> A1;
  A1 << 1, 2, 3, -1, -1, 0, 0, 1, 2, 1, 1, 2;
  Eigen::Vector4d b_lb(4, -std::numeric_limits<double>::infinity(), -20,
                       -std::numeric_limits<double>::infinity());
  Eigen::Vector4d b_ub(std::numeric_limits<double>::infinity(), -1, 100,
                       std::numeric_limits<double>::infinity());
  prog.AddLinearConstraint(A1, b_lb, b_ub);
  // This test also handles linear equality constraint
  prog.AddLinearEqualityConstraint(Eigen::RowVector3d(3, 1, 3), 3);

  RunSolver(&prog, solver);

  Eigen::VectorXd expected(3);
  expected << 0, 1, 2.0 / 3.0;

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
void TestQuadraticProgram2(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(5);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(5, 5, 0.0);
  Eigen::VectorXd Qdiag = Eigen::VectorXd::Constant(5, 0.0);
  Qdiag << 5.5, 6.5, 6.0, 5.3, 7.5;
  Q = Qdiag.asDiagonal();
  Q(2, 3) = 0.2;

  Eigen::VectorXd b = Eigen::VectorXd::Constant(5, 0.0);

  b << 3.2, 1.3, 5.6, 9.0, 1.2;

  prog.AddQuadraticCost(Q, b);

  RunSolver(&prog, solver);

  // Exact solution.
  Eigen::MatrixXd Q_symmetric = 0.5 * (Q + Q.transpose());
  Eigen::VectorXd expected = -Q_symmetric.colPivHouseholderQr().solve(b);

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
void TestQuadraticProgram3(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  const DecisionVariableView x = prog.AddContinuousVariables(6, "x");
  Eigen::MatrixXd Q1 = Eigen::MatrixXd::Constant(4, 4, 0.0);
  Eigen::VectorXd Q1diag = Eigen::VectorXd::Constant(4, 0.0);
  Q1diag << 5.5, 6.5, 6.0, 7.0;
  Q1 = Q1diag.asDiagonal();
  Q1(1, 2) = 0.1;

  Eigen::MatrixXd Q2 = Eigen::MatrixXd::Constant(4, 4, 0.0);
  Eigen::VectorXd Q2diag = Eigen::VectorXd::Constant(4, 0.0);
  Q2diag << 7.0, 2.2, 1.1, 1.3;
  Q2 = Q2diag.asDiagonal();
  Q2(0, 2) = -0.02;

  Eigen::VectorXd b1 = Eigen::VectorXd::Constant(4, 0.0);
  b1 << 3.1, -1.4, -5.6, 0.6;
  Eigen::VectorXd b2 = Eigen::VectorXd::Constant(4, 0.0);
  b2 << 2.3, -5.8, 6.7, 2.3;

  prog.AddQuadraticCost(Q1, b1, {x.head(2), x.segment(2, 2)});
  prog.AddQuadraticCost(Q2, b2, {x.segment(2, 2), x.segment(4, 2)});

  Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(6, 6, 0.0);
  Q.topLeftCorner(4, 4) = Q1;
  Q.bottomRightCorner(4, 4) += Q2;
  Eigen::MatrixXd Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Q = 0.5 * (Q + Q_transpose);

  Eigen::VectorXd b = Eigen::VectorXd::Constant(6, 0.0);
  b.topRows(4) = b1;
  b.bottomRows(4) += b2;

  // Exact solution.
  Eigen::VectorXd expected = -Q.ldlt().solve(b);

  RunSolver(&prog, solver);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Test the simple QP
// min x(0)^2 + x(1)^2 + 2 * x(2)^2
// s.t x(0) +   x(1) = 1
//     x(0) + 2*x(2) = 2
// The optimal solution should be
// x(0) = 4/5, x(1) = 1/5, x(2) = 3/5
void TestQuadraticProgram4(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3);

  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  Q(2, 2) = 2.0;
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  prog.AddQuadraticCost(Q, b);
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 1),
                                   Vector1d::Constant(1), {x(0), x(1)});
  prog.AddLinearEqualityConstraint(Eigen::RowVector2d(1, 2),
                                   Vector1d::Constant(2), {x(0), x(2)});

  RunSolver(&prog, solver);
  EXPECT_TRUE(CompareMatrices(Eigen::Vector3d(0.8, 0.2, 0.6), x.value(), 1e-9,
                              MatrixCompareType::absolute));
}

/// Solve a series of QPs with the objective being the Euclidean distance
/// from a point which moves along the unit circle (L2 ball), but
/// constrained to lie inside the L1 ball.  Implemented in 2D, so that the
/// active set moves along 4 faces of the L1 ball.
void TestQPonUnitBallExample(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Eigen::Vector2d x_desired;
  x_desired << 1.0, 0.0;
  auto objective = prog.AddQuadraticErrorCost(Q, x_desired);

  Eigen::Matrix2d A;
  A << 1.0, 1.0, -1.0, 1.0;
  Eigen::Vector2d ub = Eigen::Vector2d::Constant(1.0);
  Eigen::Vector2d lb = Eigen::Vector2d::Constant(-1.0);
  auto constraint = prog.AddLinearConstraint(A, lb, ub);
  Eigen::Vector2d x_expected;

  const int N = 40;  // number of points to test around the circle
  for (int i = 0; i < N; i++) {
    double theta = 2.0 * M_PI * i / N;
    x_desired << sin(theta), cos(theta);
    objective->UpdateQuadraticAndLinearTerms(2.0 * Q, -2.0 * Q * x_desired);

    if (theta <= M_PI_2) {
      // simple lagrange multiplier problem:
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=1
      x_expected << (x_desired(0) - x_desired(1) + 1.0) / 2.0,
          (x_desired(1) - x_desired(0) + 1.0) / 2.0;
    } else if (theta <= M_PI) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=1
      x_expected << (x_desired(0) + x_desired(1) + 1.0) / 2.0,
          (x_desired(0) + x_desired(1) - 1.0) / 2.0;
    } else if (theta <= 3.0 * M_PI_2) {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x+y=-1
      x_expected << (x_desired(0) - x_desired(1) - 1.0) / 2.0,
          (x_desired(1) - x_desired(0) - 1.0) / 2.0;
    } else {
      // min (x-x_d)^2 + (y-y_d)^2 s.t. x-y=-1
      x_expected << (x_desired(0) + x_desired(1) - 1.0) / 2.0,
          (x_desired(0) + x_desired(1) + 1.0) / 2.0;
    }

    RunSolver(&prog, solver);

    EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-4,
                                MatrixCompareType::absolute));
  }

  // provide some test coverage for changing Q
  //
  {
    // now 2(x-xd)^2 + (y-yd)^2 s.t. x+y=1
    x_desired << 1.0, 1.0;
    Q(0, 0) = 2.0;
    objective->UpdateQuadraticAndLinearTerms(2.0 * Q, -2.0 * Q * x_desired);

    x_expected << 2.0 / 3.0, 1.0 / 3.0;

    SolutionResult result = SolutionResult::kUnknownError;

    prog.SetSolverOption("GUROBI", "BarConvTol", 1E-9);
    ASSERT_NO_THROW(result = prog.Solve());
    EXPECT_EQ(result, SolutionResult::kSolutionFound);

    EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-5,
                                MatrixCompareType::absolute));
  }
}

////////////////////////////////////////
//// Second order conic program ////////
////////////////////////////////////////

/*
 * This test is taken from
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_ell_sep.html
 * The goal is to find a hyperplane, that separates two ellipsoids
 * E1 = x1 + R1 * u1, u1' * u1<=1
 * E2 = x2 + R2 * u2, u2' * u2<=1
 * A hyperplane a' * x = b separates these two ellipsoids, if and only if for
 * SOCP p* = min t1 + t2
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
template <typename DerivedX1, typename DerivedX2, typename DerivedR1,
          typename DerivedR2>
void RunEllipsoidsSeparation(const Eigen::MatrixBase<DerivedX1>& x1,
                             const Eigen::MatrixBase<DerivedX2>& x2,
                             const Eigen::MatrixBase<DerivedR1>& R1,
                             const Eigen::MatrixBase<DerivedR2>& R2,
                             const MathematicalProgramSolverInterface& solver) {
  DRAKE_ASSERT(x1.cols() == 1);
  DRAKE_ASSERT(x2.cols() == 1);
  DRAKE_ASSERT(x1.rows() == x2.rows());
  DRAKE_ASSERT(x1.rows() == R1.rows());
  DRAKE_ASSERT(x2.rows() == R2.rows());

  MathematicalProgram prog;
  const int kXdim = x1.rows();
  auto t = prog.AddContinuousVariables(2, "t");
  auto a = prog.AddContinuousVariables(kXdim, "a");
  // R1a = R1' * a
  // R2a = R2' * a
  auto R1a = prog.AddContinuousVariables(R1.cols(), "R1a");
  auto R2a = prog.AddContinuousVariables(R2.cols(), "R2a");
  Eigen::MatrixXd R1_transpose = R1.transpose();
  Eigen::MatrixXd R2_transpose = R2.transpose();
  Eigen::MatrixXd A_R1a(R1.cols(), R1.cols() + R1.rows());
  A_R1a.block(0, 0, R1.cols(), R1.cols()) =
      Eigen::MatrixXd::Identity(R1.cols(), R1.cols());
  A_R1a.block(0, R1.cols(), R1.cols(), R1.rows()) = -R1_transpose;
  Eigen::MatrixXd A_R2a(R2.cols(), R2.cols() + R2.rows());
  A_R2a.block(0, 0, R2.cols(), R2.cols()) =
      Eigen::MatrixXd::Identity(R2.cols(), R2.cols());
  A_R2a.block(0, R2.cols(), R2.cols(), R2.rows()) = -R2_transpose;
  Eigen::VectorXd b_R1a = Eigen::VectorXd::Zero(R1.cols());
  Eigen::VectorXd b_R2a = Eigen::VectorXd::Zero(R2.cols());
  prog.AddLinearEqualityConstraint(A_R1a, b_R1a, {R1a, a});
  prog.AddLinearEqualityConstraint(A_R2a, b_R2a, {R2a, a});

  // a'*(x2 - x1) = 1
  prog.AddLinearEqualityConstraint((x2 - x1).transpose(), 1.0, {a});

  // Add cost
  auto cost = prog.AddLinearCost(Eigen::RowVector2d(1.0, 1.0), {t});

  // Add Lorentz cones
  auto lorentz_cone1 = prog.AddLorentzConeConstraint({t(0), R1a});
  auto lorentz_cone2 = prog.AddLorentzConeConstraint({t(1), R2a});

  RunSolver(&prog, solver);

  // Check the solution.
  // First check if each constraint is satisfied.
  EXPECT_TRUE(CompareMatrices(R1a.value(), R1_transpose * a.value(), 1e-7,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(R2a.value(), R2_transpose * a.value(), 1e-7,
                              MatrixCompareType::absolute));
  EXPECT_NEAR(t.value().coeff(0), R1a.value().norm(), 1e-6);
  EXPECT_NEAR(t.value().coeff(1), R2a.value().norm(), 1e-6);
  EXPECT_TRUE(CompareMatrices((x2 - x1).transpose() * a.value(),
                              drake::Vector1d(1.0), 1e-8,
                              MatrixCompareType::absolute));

  // Now check if the solution is meaningful, that it really finds a separating
  // hyperplane.
  // The separating hyperplane exists if and only if p* <= 1
  double p_star = t.value().coeff(0) + t.value().coeff(1);
  bool is_separated = p_star <= 1.0;
  double t1 = t.value().coeff(0);
  double t2 = t.value().coeff(1);
  if (is_separated) {
    // Then the hyperplane a' * x = 0.5 * (a'*x1 + t1 + a'*x2 - t2)
    double b1 = a.value().transpose() * x1 + t1;
    double b2 = a.value().transpose() * x2 - t2;
    double b = 0.5 * (b1 + b2);
    // Verify that b - a'*x1 >= |R1' * a|
    //             a'*x2 - b >= |R2' * a|
    EXPECT_TRUE(b - a.value().transpose() * x1 >=
                (R1_transpose * a.value()).norm());
    EXPECT_TRUE(a.value().transpose() * x2 - b >=
                (R2_transpose * a.value()).norm());
  } else {
    // Now solve another SOCP to find a point y in the intersecting region
    // y = x1 + R1*u1
    // y = x2 + R2*u2
    // 1 >= |u1|
    // 1 >= |u2|
    MathematicalProgram prog_intersect;
    auto u1 = prog_intersect.AddContinuousVariables(R1.cols(), "u1");
    auto u2 = prog_intersect.AddContinuousVariables(R2.cols(), "u2");
    auto y = prog_intersect.AddContinuousVariables(kXdim, "y");

    auto slack = prog_intersect.AddContinuousVariables(1, "slack");
    prog_intersect.AddBoundingBoxConstraint(1, 1, slack);

    prog_intersect.AddLorentzConeConstraint({slack, u1});
    prog_intersect.AddLorentzConeConstraint({slack, u2});

    // Add constraint y = x1 + R1*u1
    //                y = x2 + R2*u2
    Eigen::MatrixXd A1(y.size(), y.size() + R1.cols());
    A1.block(0, 0, y.size(), y.size()) =
        Eigen::MatrixXd::Identity(y.size(), y.size());
    A1.block(0, y.size(), y.size(), R1.cols()) = -R1;
    Eigen::MatrixXd A2(y.size(), y.size() + R2.cols());
    A2.block(0, 0, y.size(), y.size()) =
        Eigen::MatrixXd::Identity(y.size(), y.size());
    A2.block(0, y.size(), y.size(), R2.cols()) = -R2;
    prog_intersect.AddLinearEqualityConstraint(A1, x1, {y, u1});
    prog_intersect.AddLinearEqualityConstraint(A2, x2, {y, u2});

    RunSolver(&prog_intersect, solver);

    // Check if the constraints are satisfied
    EXPECT_LE(u1.value().norm(), 1);
    EXPECT_LE(u2.value().norm(), 1);
    EXPECT_TRUE(CompareMatrices(y.value(), x1 + R1 * u1.value(), 1e-8,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(y.value(), x2 + R2 * u2.value(), 1e-8,
                                MatrixCompareType::absolute));
  }
}

/*
 * This example is taken from
 * https://inst.eecs.berkeley.edu/~ee127a/book/login/exa_qp_as_socp.html
 * For a quadratic program
 * 0.5 * x' * Q * x + c' * x
 * s.t b_lb <= A * x <= b_ub
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
 */
template <typename DerivedQ, typename DerivedC, typename DerivedA,
          typename DerivedBlower, typename DerivedBupper>
void SolveQPasSOCP(const Eigen::MatrixBase<DerivedQ>& Q,
                   const Eigen::MatrixBase<DerivedC>& c,
                   const Eigen::MatrixBase<DerivedA>& A,
                   const Eigen::MatrixBase<DerivedBlower>& b_lb,
                   const Eigen::MatrixBase<DerivedBupper>& b_ub,
                   const MathematicalProgramSolverInterface& solver) {
  DRAKE_ASSERT(Q.rows() == Q.cols());
  Eigen::MatrixXd Q_symmetric = 0.5 * (Q + Q.transpose());
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

  prog_socp.AddBoundingBoxConstraint(2.0, 2.0, z);
  prog_socp.AddRotatedLorentzConeConstraint({y, z, w});

  Eigen::LLT<Eigen::MatrixXd, Eigen::Upper> lltOfQ(Q_symmetric);
  Eigen::MatrixXd Q_sqrt = lltOfQ.matrixU();
  Eigen::MatrixXd A_w(kXdim, 2 * kXdim);
  A_w << Eigen::MatrixXd::Identity(kXdim, kXdim), -Q_sqrt;
  prog_socp.AddLinearEqualityConstraint(A_w, Eigen::VectorXd::Zero(kXdim),
                                        {w, x_socp});

  prog_socp.AddLinearConstraint(A, b_lb, b_ub, {x_socp});

  std::shared_ptr<LinearConstraint> cost_socp1(new LinearConstraint(
      c.transpose(), drake::Vector1d(-std::numeric_limits<double>::infinity()),
      drake::Vector1d(std::numeric_limits<double>::infinity())));
  prog_socp.AddCost(cost_socp1, {x_socp});
  prog_socp.AddLinearCost(drake::Vector1d(1.0), {y});
  RunSolver(&prog_socp, solver);
  double objective_value_socp =
      c.transpose() * x_socp.value() + y.value().coeff(0);

  // Check the solution
  EXPECT_NEAR(2 * y.value().coeff(0), w.value().squaredNorm(), 1E-6);
  EXPECT_TRUE(CompareMatrices(w.value(), Q_sqrt * x_socp.value(), 1e-6,
                              MatrixCompareType::absolute));
  EXPECT_GE(y.value().coeff(0), 0);
  EXPECT_TRUE(CompareMatrices(w.value(), Q_sqrt * x_socp.value(), 1E-6,
                              MatrixCompareType::absolute));

  // Now solve the problem as a QP.
  MathematicalProgram prog_qp;
  auto x_qp = prog_qp.AddContinuousVariables(kXdim, "x");
  prog_qp.AddQuadraticCost(Q, c, {x_qp});
  prog_qp.AddLinearConstraint(A, b_lb, b_ub, {x_qp});
  RunSolver(&prog_qp, solver);
  double objective_value_qp =
      0.5 * (x_qp.value().transpose() * Q * x_qp.value()).coeff(0, 0) +
      c.transpose() * x_qp.value();

  // TODO(hongkai.dai@tri.global): tighten the tolerance. socp does not really
  // converge to true optimal yet.
  EXPECT_TRUE(CompareMatrices(x_qp.value(), x_socp.value(), 2e-4,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(std::abs(objective_value_qp - objective_value_socp) < 1E-6);
}

void TestQPasSOCP(const MathematicalProgramSolverInterface& solver) {
  // Solve an un-constrained QP
  Eigen::MatrixXd Q = Eigen::Matrix2d::Identity();
  Eigen::VectorXd c = Eigen::Vector2d::Ones();
  Eigen::MatrixXd A = Eigen::RowVector2d(0, 0);
  Eigen::VectorXd b_lb =
      Eigen::VectorXd::Constant(1, -std::numeric_limits<double>::infinity());
  Eigen::VectorXd b_ub =
      Eigen::VectorXd::Constant(1, std::numeric_limits<double>::infinity());
  SolveQPasSOCP(Q, c, A, b_lb, b_ub, solver);

  // Solve a constrained QP
  Q = Eigen::Matrix3d::Zero();
  Q(0, 0) = 1.0;
  Q(1, 1) = 1.3;
  Q(2, 2) = 2.0;
  Q(1, 2) = 0.01;
  Q(0, 1) = -0.2;
  c = Eigen::Vector3d::Zero();
  c(0) = -1.0;
  c(1) = -2.0;
  c(2) = 1.2;

  A = Eigen::Matrix<double, 2, 3>::Zero();
  A << 1, 0, 2, 0, 1, 3;
  b_lb = Eigen::Vector2d::Zero();
  b_lb(0) = -1;
  b_lb(1) = -2;
  b_ub = Eigen::Vector2d::Zero();
  b_ub(0) = 2;
  b_ub(1) = 4;
  SolveQPasSOCP(Q, c, A, b_lb, b_ub, solver);
}

/*
 * This example is taken from the paper
 * Applications of second-order cone programming
 * By M.S.Lobo, L.Vandenberghe, S.Boyd and H.Lebret,
 * Section 3.6
 * http://www.seas.ucla.edu/~vandenbe/publications/socp.pdf
 * The problem tries to find the equilibrium state of a mechanical
 * system, which consists of N nodes at position (x1,y1), (x2,y2), ..., (xN,yN)
 * in R2.
 * The nodes are connected by springs with given coefficient.
 * The spring generate force when it is stretched,
 * but not when it is compressed.
 * Namely, the spring force is
 * (spring_length - spring_rest_length) * spring_stiffness,
 * if spring_length >= spring_rest_length;
 * otherwise the spring force is zero.
 * weight_i is the mass * gravity_acceleration
 * of the i'th link.
 * The equilibrium point is obtained when the total energy is minimized
 * namely min sum_i weight_i * yi + stiffness/2 * t_i^2
 *        s.t  sqrt((x(i) - x(i+1))^2 + (y(i) - y(i+1))^2) - spring_rest_length
 * <= t_i
 *             0 <= t_i
 *             (x1,y1) = end_pos1
 *             (xN,yN) = end_pos2
 * By introducing a slack variable z >= t_1^2 + ... + t_(N-1)^2, the problem
 * becomes
 * an SOCP, with both Lorentz cone and rotated Lorentz cone constraints
 *
 */
void FindSpringEquilibrium(const Eigen::VectorXd& weight,
                           double spring_rest_length, double spring_stiffness,
                           const Eigen::Vector2d& end_pos1,
                           const Eigen::Vector2d& end_pos2,
                           const MathematicalProgramSolverInterface& solver) {
  int num_nodes = weight.rows();
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(num_nodes, "x");
  auto y = prog.AddContinuousVariables(num_nodes, "y");
  auto t = prog.AddContinuousVariables(num_nodes - 1, "t");
  prog.AddBoundingBoxConstraint(end_pos1, end_pos1, {x(0), y(0)});
  prog.AddBoundingBoxConstraint(end_pos2, end_pos2,
                                {x(num_nodes - 1), y(num_nodes - 1)});
  prog.AddBoundingBoxConstraint(
      Eigen::VectorXd::Zero(num_nodes - 1),
      Eigen::VectorXd::Constant(num_nodes - 1,
                                std::numeric_limits<double>::infinity()),
      {t});
  auto t_plus_l0 = prog.AddContinuousVariables(
      num_nodes - 1,
      "t_plus_l0");  // The slack variable equals to t_i + spring_rest_length
  Eigen::MatrixXd A1(num_nodes - 1, 2 * (num_nodes - 1));
  A1 << Eigen::MatrixXd::Identity(num_nodes - 1, num_nodes - 1),
      -Eigen::MatrixXd::Identity(num_nodes - 1, num_nodes - 1);
  prog.AddLinearEqualityConstraint(
      A1, Eigen::VectorXd::Constant(num_nodes - 1, spring_rest_length),
      {t_plus_l0, t});
  // x_diff(i) = x(i+1) - x(i)
  // y_diff(i) = y(i+1) - y(i)
  auto x_diff = prog.AddContinuousVariables(num_nodes - 1, "xd");
  auto y_diff = prog.AddContinuousVariables(num_nodes - 1, "yd");
  Eigen::MatrixXd A2(num_nodes - 1, 2 * num_nodes - 1);
  A2.setZero();
  A2.block(0, 0, num_nodes - 1, num_nodes - 1) =
      -Eigen::MatrixXd::Identity(num_nodes - 1, num_nodes - 1);
  A2.block(0, 1, num_nodes - 1, num_nodes - 1) +=
      Eigen::MatrixXd::Identity(num_nodes - 1, num_nodes - 1);
  A2.block(0, num_nodes, num_nodes - 1, num_nodes - 1) =
      -Eigen::MatrixXd::Identity(num_nodes - 1, num_nodes - 1);
  prog.AddLinearEqualityConstraint(A2, Eigen::VectorXd::Zero(num_nodes - 1),
                                   {x, x_diff});
  prog.AddLinearEqualityConstraint(A2, Eigen::VectorXd::Zero(num_nodes - 1),
                                   {y, y_diff});

  // sqrt((x(i)-x(i+1))^2 + (y(i) - y(i+1))^2) <= ti + spring_rest_length
  for (int i = 0; i < num_nodes - 1; ++i) {
    prog.AddLorentzConeConstraint({t_plus_l0(i), x_diff(i), y_diff(i)});
  }

  // Add constraint z >= t_1^2 + .. + t_(N-1)^2
  auto z = prog.AddContinuousVariables(2, "z");
  prog.AddBoundingBoxConstraint(1, 1, z(0));
  prog.AddRotatedLorentzConeConstraint({z, t});

  prog.AddLinearCost(drake::Vector1d(spring_stiffness / 2), {z(1)});
  prog.AddLinearCost(weight.transpose(), {y});

  RunSolver(&prog, solver);

  std::string solver_name;
  int solver_result;
  prog.GetSolverResult(&solver_name, &solver_result);
  double precision = 1e-3;
  // The precision of Gurobi solver is not as good as Mosek, in
  // this problem.
  if (solver_name == "Gurobi") {
    precision = 2e-2;
  }
  for (int i = 0; i < num_nodes - 1; ++i) {
    Eigen::Vector2d spring(x.value()(i + 1) - x.value()(i),
                           y.value()(i + 1) - y.value()(i));
    if (spring.norm() < spring_rest_length) {
      EXPECT_LE(t.value()(i), 1E-3);
      EXPECT_GE(t.value()(i), 0 - 1E-10);
    } else {
      EXPECT_TRUE(std::abs(spring.norm() - spring_rest_length - t.value()(i)) <
                  1E-3);
    }
  }
  EXPECT_TRUE(std::abs(z.value()(1) - t.value().squaredNorm()) < 1E-3);
  // Now test equilibrium.
  for (int i = 1; i < num_nodes - 1; i++) {
    Eigen::Vector2d left_spring(x.value().coeff(i - 1) - x.value().coeff(i),
                                y.value().coeff(i - 1) - y.value().coeff(i));
    Eigen::Vector2d left_spring_force;
    double left_spring_length = left_spring.norm();
    if (left_spring_length < spring_rest_length) {
      left_spring_force.setZero();
    } else {
      left_spring_force = (left_spring_length - spring_rest_length) *
                          spring_stiffness * left_spring / left_spring_length;
    }
    Eigen::Vector2d right_spring(x.value().coeff(i + 1) - x.value().coeff(i),
                                 y.value().coeff(i + 1) - y.value().coeff(i));
    Eigen::Vector2d right_spring_force;
    double right_spring_length = right_spring.norm();
    if (right_spring_length < spring_rest_length) {
      right_spring_force.setZero();
    } else {
      right_spring_force = (right_spring_length - spring_rest_length) *
                           spring_stiffness * right_spring /
                           right_spring_length;
    }
    Eigen::Vector2d weight_i(0, -weight(i));
    EXPECT_TRUE(CompareMatrices(
        weight_i + left_spring_force + right_spring_force,
        Eigen::Vector2d::Zero(), precision, MatrixCompareType::absolute));
  }
}

void TestFindSpringEquilibrium(
    const MathematicalProgramSolverInterface& solver) {
  Eigen::VectorXd weight(5);
  weight << 1, 2, 3, 2.5, 4;
  double spring_rest_length = 0.2;
  double spring_stiffness_coefficient = 10;
  Eigen::Vector2d end_pos1(0, 1);
  Eigen::Vector2d end_pos2(1, 0.9);

  FindSpringEquilibrium(weight, spring_rest_length,
                        spring_stiffness_coefficient, end_pos1, end_pos2,
                        solver);
}

void GetLinearProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverToListIfAvailable("Gurobi", solvers);
  AddSolverToListIfAvailable("Mosek", solvers);
}

void GetQuadraticProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverToListIfAvailable("Gurobi", solvers);
  AddSolverToListIfAvailable("Mosek", solvers);
}

void GetSecondOrderConicProgramSolvers(
    std::list<std::unique_ptr<MathematicalProgramSolverInterface>>* solvers) {
  AddSolverToListIfAvailable("Gurobi", solvers);
  AddSolverToListIfAvailable("Mosek", solvers);
}
}  // namespace

GTEST_TEST(TestConvexOptimization, TestLinearProgramFeasibility) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestLinearProgramFeasibility(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestLinearProgram0) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestLinearProgram0(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestLinearProgram1) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestLinearProgram1(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestLinearProgram2) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetLinearProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestLinearProgram2(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgram0) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQuadraticProgram0(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgram1) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQuadraticProgram1(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgram2) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQuadraticProgram2(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgram3) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQuadraticProgram3(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgram4) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQuadraticProgram4(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQuadraticProgramUnitBall) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetQuadraticProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQPonUnitBallExample(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestEllipsoidsSeparation0) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    Eigen::VectorXd x1 = Eigen::Vector3d::Zero();
    Eigen::VectorXd x2 = Eigen::Vector3d::Zero();
    x2(0) = 2.0;
    Eigen::MatrixXd R1 = 0.5 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R2 = Eigen::Matrix3d::Identity();
    RunEllipsoidsSeparation(x1, x2, R1, R2, *solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestEllipsoidsSeparation1) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    // Test if we can find a common point for two intersecting balls.
    Eigen::VectorXd x1 = Eigen::Vector3d::Zero();
    Eigen::VectorXd x2 = Eigen::Vector3d::Zero();
    x2(0) = 1.0;
    Eigen::MatrixXd R1 = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R2 = Eigen::Matrix3d::Identity();
    RunEllipsoidsSeparation(x1, x2, R1, R2, *solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestEllipsoidsSeparation2) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    // Test two ellipsoids
    Eigen::VectorXd x1 = Eigen::Vector2d(1.0, 0.2);
    Eigen::VectorXd x2 = Eigen::Vector2d(0.5, 0.4);
    Eigen::MatrixXd R1 = Eigen::Matrix2d::Zero();
    R1 << 0.1, 0.6, 0.2, 1.3;
    Eigen::MatrixXd R2 = Eigen::Matrix2d::Zero();
    R2 << -0.4, 1.5, 1.7, 0.3;
    RunEllipsoidsSeparation(x1, x2, R1, R2, *solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestEllipsoidsSeparation3) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    // Test another two ellipsoids
    Eigen::VectorXd x1 = Eigen::Vector3d(1.0, 0.2, 0.8);
    Eigen::VectorXd x2 = Eigen::Vector3d(3.0, -1.5, 1.9);
    Eigen::MatrixXd R1 = Eigen::Matrix3d::Zero();
    R1 << 0.2, 0.4, 0.2, -0.2, -0.1, 0.3, 0.2, 0.1, 0.1;
    Eigen::MatrixXd R2 = Eigen::Matrix<double, 3, 2>::Zero();
    R2 << 0.1, 0.2, -0.1, 0.01, -0.2, 0.1;
    RunEllipsoidsSeparation(x1, x2, R1, R2, *solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestQPasSOCP) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestQPasSOCP(*solver);
  }
}

GTEST_TEST(TestConvexOptimization, TestFindSpringEquilibrium) {
  std::list<std::unique_ptr<MathematicalProgramSolverInterface>> solvers;
  GetSecondOrderConicProgramSolvers(&solvers);
  for (const auto& solver : solvers) {
    TestFindSpringEquilibrium(*solver);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
