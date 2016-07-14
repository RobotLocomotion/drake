#include <typeinfo>

#include "drake/common/drake_assert.h"
//#include "drake/solvers/IpoptSolver.h"
#include "drake/solvers/MathematicalProgram.h"
//#include "drake/solvers/NloptSolver.h"
#include "drake/solvers/Optimization.h"
#include "drake/solvers/SnoptSolver.h"
#include "drake/solvers/GurobiSolver.h"
#include "drake/util/Polynomial.h"
#include "drake/util/eigen_matrix_compare.h"
#include "drake/util/testUtil.h"
#include "gtest/gtest.h"

#include "gurobi_c.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::util::MatrixCompareType;

namespace drake {
namespace solvers {
namespace {

void RunQuadraticProgram(OptimizationProblem &prog,
                         std::function<void(void)> test_func) {

  GurobiSolver gurobi_solver;

  // tests wont fail with absence of Gurobi
  if (gurobi_solver.available()) {
     SolutionResult result = SolutionResult::kUnknownError;
    ASSERT_NO_THROW(result = gurobi_solver.Solve(prog));
    EXPECT_EQ(result, SolutionResult::kSolutionFound);
    EXPECT_NO_THROW(test_func());
  }
}


/// Simple test from the Gurobi documentation.
//  min    x^2 + x*y + y^2 + y*z + z^2 + 2 x
//  subj to  x + 2 y + 3 z >= 4
//           x +   y       >= 1
GTEST_TEST(testGurobi, gurobiQPExample1) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(3);

  Eigen::MatrixXd Q = Eigen::Matrix<double, 3, 3>::Identity();
  Q(0,1) = 1;
  Q(1,2) = 1;
  Q(1,0) = 1;
  Q(2,1) = 1;

  prog.AddBoundingBoxConstraint(MatrixXd::Constant(3, 1, 0),
                                MatrixXd::Constant(3, 1, std::numeric_limits<double>::infinity()));

  Eigen::VectorXd b(3);
  b << 2.0, 0.0, 0.0;

  prog.AddQuadraticCost(Q, b);

  VectorXd constraint(3);
  constraint << 1, 2, 3;
  prog.AddLinearConstraint(constraint.transpose(),
      Drake::Vector1d::Constant(4),
      Drake::Vector1d::Constant(std::numeric_limits<double>::infinity()));

  VectorXd constraint2(3);
  constraint2 << 1, 1, 0;
  prog.AddLinearConstraint(constraint2.transpose(),
      Drake::Vector1d::Constant(1),
      Drake::Vector1d::Constant(std::numeric_limits<double>::infinity()));

  VectorXd expected(3);
  expected << 0, 1, 2.0/3.0;

  RunQuadraticProgram(prog,[&]() {
  EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-8,
                              MatrixCompareType::absolute));
}
);
}

/// Closed form (exact) solution test of QP problem.
// For any Positive Semi Definite square matrix Q :
// min x'Qx + bx = -Q^(-1) * b
GTEST_TEST(testGurobi, convexQPExample) {
  OptimizationProblem prog;
  auto x = prog.AddContinuousVariables(5);
  MatrixXd Q = 10 * Eigen::VectorXd::Random(5).asDiagonal();
  Q = Q.array().abs().matrix();

  Eigen::VectorXd b = 10 * Eigen::VectorXd::Random(5);

  prog.AddQuadraticCost(Q, b);

  // exact solution
  VectorXd expected = -Q.ldlt().solve(b);

  RunQuadraticProgram(prog,[&]() {
  EXPECT_TRUE(CompareMatrices(x.value(), expected, 1e-8,
                              MatrixCompareType::absolute));
}
);
}

} // close namespace
} // close namespace solvers
} // close namespace drake