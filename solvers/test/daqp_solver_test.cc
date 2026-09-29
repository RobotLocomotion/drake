#include "drake/solvers/daqp_solver.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {

GTEST_TEST(DaqpSolverTest, DifferentialIkCollisionConstraint) {
  // The clipped collision example from Drake's differential IK system test:
  // desired velocity (-8, 0.5), with collision constraint v_x >= -4.
  MathematicalProgram prog;
  const auto v = prog.NewContinuousVariables<2>("v");
  prog.AddQuadraticCost(2 * Eigen::Matrix2d::Identity(),
                        Eigen::Vector2d(16, -1), v);
  const Eigen::RowVector2d A(1, 0);
  const Eigen::Vector<double, 1> lower(-4);
  const Eigen::Vector<double, 1> upper(std::numeric_limits<double>::infinity());
  const auto collision = prog.AddLinearConstraint(A, lower, upper, v);

  DaqpSolver solver;
  EXPECT_TRUE(solver.available());
  EXPECT_TRUE(solver.AreProgramAttributesSatisfied(prog));
  const auto result = solver.Solve(prog);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(v(0)), -4, 1e-7);
  EXPECT_NEAR(result.GetSolution(v(1)), 0.5, 1e-7);
  EXPECT_NEAR(result.GetDualSolution(collision)(0), 8, 1e-7);
  EXPECT_EQ(result.get_solver_details<DaqpSolver>().exitflag, 1);
}

GTEST_TEST(DaqpSolverTest, BoundsAndEquality) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddQuadraticCost(2 * Eigen::Matrix2d::Identity(),
                        Eigen::Vector2d::Zero(), x);
  const auto equality = prog.AddLinearEqualityConstraint(x(0) + x(1) == 1);
  const auto bound = prog.AddBoundingBoxConstraint(0.8, 2.0, x(0));
  DaqpSolver solver;
  const auto result = solver.Solve(prog);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(x(0)), 0.8, 1e-6);
  EXPECT_NEAR(result.GetSolution(x(1)), 0.2, 1e-6);
  EXPECT_EQ(result.GetDualSolution(equality).size(), 1);
  EXPECT_NEAR(result.GetDualSolution(bound)(0), 1.2, 1e-6);
}

GTEST_TEST(DaqpSolverTest, InfeasibleBounds) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(x(0) * x(0));
  prog.AddBoundingBoxConstraint(2, 3, x);
  prog.AddBoundingBoxConstraint(-1, 1, x);
  DaqpSolver solver;
  const auto result = solver.Solve(prog);
  EXPECT_EQ(result.get_solution_result(),
            SolutionResult::kInfeasibleConstraints);
}

GTEST_TEST(DaqpSolverTest, SemidefiniteLeastSquaresCost) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddQuadraticCost((x(0) + x(1) - 1) * (x(0) + x(1) - 1));
  prog.AddBoundingBoxConstraint(0, 2, x);
  const auto result = DaqpSolver().Solve(prog);
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(x(0)) + result.GetSolution(x(1)), 1, 1e-6);
}

GTEST_TEST(DaqpSolverTest, SolverRegistryAndCapabilities) {
  const auto solver = MakeSolver(DaqpSolver::id());
  EXPECT_EQ(solver->solver_id(), DaqpSolver::id());
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(-x(0) * x(0), false);
  EXPECT_FALSE(DaqpSolver::ProgramAttributesSatisfied(prog));
}

}  // namespace
}  // namespace solvers
}  // namespace drake
