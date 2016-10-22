#include "drake/solvers/test/optimization_program_examples.h"

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"

namespace drake{
namespace solvers {
namespace test {
// Test a simple linear programming problem with zero cost, i.e. a feasibility
// problem
//    0 <= x0 + 2x1 + 3x2 <= 10
// -inf <=       x1 - 2x2 <= 3
//           x1 >= 1
void testLinearProgramFeasibility(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(3, "x");
  Eigen::Matrix<double, 2, 3> A;
  A<< 1, 2, 3,
      0, 1, -2;
  Eigen::Vector2d b_lb(0, -std::numeric_limits<double>::infinity());
  Eigen::Vector2d b_ub(10, 3);
  prog.AddLinearConstraint(A, b_lb, b_ub);
  prog.AddBoundingBoxConstraint(drake::Vector1d(1.0), drake::Vector1d(std::numeric_limits<double>::infinity()), {x(1)});

  SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

  Eigen::Vector2d A_times_x = A * x.value();
  EXPECT_GE(A_times_x(0), 0 - 1e-10);
  EXPECT_LE(A_times_x(0), 10 + 1e-10);
  EXPECT_LE(A_times_x(1), 3 + 1E-10);
  EXPECT_GE(x.value().coeff(1), 1 - 1E-10);
}

// Adapt from the linear programming example
// http://cvxopt.org/examples/tutorial/lp.html
// Solve the following linear program
// min  2x0 + x1
// s.t -x0 + x1 <= 1
//      x0 + x1 >= 2
//      x0 >= 0
//      x0 - 2x1 <= 4
//      x1 >= 2
// The optimal solution is x0 = 1, x1 = 2
void testLinearProgram0(const MathematicalProgramSolverInterface& solver) {
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

  SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

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
void testLinearProgram1(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(2, "x");
  prog.AddLinearCost(Eigen::RowVector2d(1.0, -2.0));
  prog.AddBoundingBoxConstraint(Eigen::Vector2d(0, -1), Eigen::Vector2d(2, 4));

  SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

  EXPECT_TRUE(x.value().isApprox(Eigen::Vector2d(0, 4)));
}

// Test a simple linear programming problem
// Adapt from http://docs.mosek.com/7.1/capi/Linear_optimization.html
// min -3x0 - x1 - 5x2 - x3
// s.t     3x0 +  x1 + 2x2        = 30
//         2x0 +  x1 + 3x2 +  x3 >= 15
//               2x1       + 3x3 <= 25
// -inf <=  x0 + 2x1       + x3  <= inf
// -100 <=  x0       + 2x2       <= 40
//           0 <= x0 <= inf
//           0 <= x1 <= 10
//           0 <= x2 <= inf
//           0 <= x3 <= inf
// The optimal solution is at (0, 0, 15, 25/3)
void testLinearProgram2(const MathematicalProgramSolverInterface& solver) {
  MathematicalProgram prog;
  auto x = prog.AddContinuousVariables(4, "x");
  // We deliberately break the cost to c1' * [x0;x1;x2] + c2'*[x2;x3] here
  // to test adding multiple costs.
  Eigen::RowVector3d c1(-3, -1, -4);
  Eigen::RowVector2d c2(-1, -1);

  prog.AddLinearCost(c1, {x(0), x(1), x(2)});
  prog.AddLinearCost(c2, {x(2), x(3)});

  Eigen::RowVector3d a1(3, 1, 2);
  prog.AddLinearEqualityConstraint(a1, drake::Vector1d(30), {x(0), x(1), x(2)});

  Eigen::Matrix<double, 4, 4> A;
  A << 2, 1, 3, 1,
  0, 2, 0, 3,
  1, 2, 0, 1,
  1, 0, 2, 0;
  Eigen::Vector4d b_lb(15, -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -100);
  Eigen::Vector4d b_ub(std::numeric_limits<double>::infinity(), 25, std::numeric_limits<double>::infinity(), 40);
  prog.AddLinearConstraint(A, b_lb, b_ub);

  prog.AddBoundingBoxConstraint(drake::Vector1d(0), drake::Vector1d(10), {x(1)});
  prog.AddBoundingBoxConstraint(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), {x(0), x(2), x(3)});

  SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

  Eigen::Vector4d x_expected(0, 0, 15, 25.0/3.0);
  EXPECT_TRUE(CompareMatrices(x.value(), x_expected, 1e-10, MatrixCompareType::absolute));

}

void testLinearPrograms(const MathematicalProgramSolverInterface& solver) {
  testLinearProgramFeasibility(solver);
  testLinearProgram0(solver);
  testLinearProgram1(solver);
  testLinearProgram2(solver);
}
} // namespace test
} // namespace solvers
} // namespace drake