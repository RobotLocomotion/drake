#include "drake/solvers/gurobi_solver.h"

#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {
namespace {

void RunQuadraticProgram(MathematicalProgram* prog) {
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

  Eigen::MatrixXd Q = Eigen::Matrix<double, 3, 3>::Identity();
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

  RunQuadraticProgram(&prog);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Closed form (exact) solution test of QP problem.
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
GTEST_TEST(testGurobi, convexQPExample) {
      MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(5);
  MatrixXd Q = MatrixXd::Constant(5, 5, 0.0);
  VectorXd Qdiag = VectorXd::Constant(5, 0.0);
  Qdiag << 5.5, 6.5, 6.0, 5.3, 7.5;
  Q = Qdiag.asDiagonal();

  Eigen::VectorXd b = VectorXd::Constant(5, 0.0);

  b << 3.2, 1.3, 5.6, 9.0, 1.2;

  prog.AddQuadraticCost(Q, b);

  // Exact solution.
  VectorXd expected = -Q.colPivHouseholderQr().solve(b);
  RunQuadraticProgram(&prog);
  EXPECT_TRUE(
      CompareMatrices(x.value(), expected, 1e-8, MatrixCompareType::absolute));
}

// Closed form (exact) solution test of QP problem.
// Added as multiple QP cost terms
// Note that for any Positive Semi Definite matrix Q :
// min 0.5x'Qx + bx = -Q^(-1) * b
// The values were chosen at random but were hardcoded
// to enable test reproducibility.
GTEST_TEST(testGurobi, convexQPMultiCostExample) {
      MathematicalProgram prog;
  const DecisionVariableView x1 = prog.AddContinuousVariables(3, "x1");
  MatrixXd Q1 = MatrixXd::Constant(3, 3, 0.0);
  VectorXd Q1diag = VectorXd::Constant(3, 0.0);
  Q1diag << 5.5, 6.5, 6.0;
  Q1 = Q1diag.asDiagonal();

  const DecisionVariableView x2 = prog.AddContinuousVariables(3, "x2");
  MatrixXd Q2 = MatrixXd::Constant(3, 3, 0.0);
  VectorXd Q2diag = VectorXd::Constant(3, 0.0);
  Q2diag << 7.0, 2.2, 1.1;
  Q2 = Q2diag.asDiagonal();

  VectorXd b1 = VectorXd::Constant(3, 0.0);
  b1 << 3.1, -1.4, -5.6;
  VectorXd b2 = VectorXd::Constant(3, 0.0);
  b2 << 2.3, -5.8, 6.7;

  prog.AddQuadraticCost(Q1, b1, {x1});
  prog.AddQuadraticCost(Q2, b2, {x2});

  MatrixXd Q = Eigen::MatrixXd::Constant(6, 6, 0.0);
  Q.topLeftCorner(3, 3) = Q1;
  Q.bottomRightCorner(3, 3) = Q2;

  VectorXd b = VectorXd::Constant(6, 0.0);
  b.topRows(3) = b1;
  b.bottomRows(3) = b2;

  // Exact solution.
  VectorXd expected = -Q.ldlt().solve(b);

  RunQuadraticProgram(&prog);
  VectorXd composed_solution = VectorXd::Constant(6, 0.0);
  composed_solution.topRows(3) = x1.value();
  composed_solution.bottomRows(3) = x2.value();
  EXPECT_TRUE(CompareMatrices(composed_solution, expected, 1e-8,
                              MatrixCompareType::absolute));
}

}  // close namespace
}  // close namespace solvers
}  // close namespace drake
