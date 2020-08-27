#include "drake/solvers/solve.h"

#include <regex>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
GTEST_TEST(SolveTest, LinearSystemSolverTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearEqualityConstraint(Eigen::Matrix2d::Identity(),
                                   Eigen::Vector2d(1, 2), x);
  auto result = Solve(prog, {}, {});
  EXPECT_TRUE(result.is_success());
  EXPECT_TRUE(
      CompareMatrices(result.get_x_val(), Eigen::Vector2d(1, 2), 1E-12));
  EXPECT_EQ(result.get_optimal_cost(), 0);
  EXPECT_EQ(result.get_solver_id(), LinearSystemSolver::id());

  // Now add an inconsistent constraint
  prog.AddLinearEqualityConstraint(x(0) + x(1), 5);
  result = Solve(prog, {}, {});
  EXPECT_FALSE(result.is_success());
  EXPECT_EQ(result.get_optimal_cost(),
            MathematicalProgram::kGlobalInfeasibleCost);
  EXPECT_EQ(result.get_solver_id(), LinearSystemSolver::id());
}

GTEST_TEST(SolveTest, TestInitialGuessAndOptions) {
  // Test with gurobi solver, which accepts both initial guess and solver
  // options.
  MathematicalProgram prog;
  auto x = prog.NewBinaryVariables<1>();
  if (GurobiSolver::is_available() &&
      ChooseBestSolver(prog) == GurobiSolver::id()) {
    SolverOptions solver_options;
    // Presolve and Heuristics would each independently solve
    // this problem inside of the Gurobi solver, but without
    // consulting the initial guess.
    solver_options.SetOption(GurobiSolver::id(), "Presolve", 0);
    solver_options.SetOption(GurobiSolver::id(), "Heuristics", 0.0);
    Eigen::VectorXd x_expected(1);
    x_expected(0) = 1;
    MathematicalProgramResult result = Solve(prog, x_expected, solver_options);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), x_expected, 1E-6));
  }
}

}  // namespace solvers
}  // namespace drake
