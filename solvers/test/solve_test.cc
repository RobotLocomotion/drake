#include "drake/solvers/solve.h"

#include <regex>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/scs_solver.h"
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

GTEST_TEST(SolveTest, ParallelLinearSystemSolverTest) {
  const int N = 10;
  std::vector<MathematicalProgram> prog_list(N);
  for (int i = 0; i < N; ++i) {
    auto x = prog_list.at(i).NewContinuousVariables<2>();
    prog_list.at(i).AddLinearEqualityConstraint(Eigen::Matrix2d::Identity(),
                                                Eigen::Vector2d(1, 2), x);
  }
  auto result_list = SolveInParallel(prog_list, 3, true);
  for (int i = 0; i < N; ++i) {
    EXPECT_TRUE(result_list.at(i)->is_success());
    EXPECT_TRUE(CompareMatrices(result_list.at(i)->get_x_val(),
                                Eigen::Vector2d(1, 2), 1E-12));
    EXPECT_EQ(result_list.at(i)->get_optimal_cost(), 0);
    EXPECT_EQ(result_list.at(i)->get_solver_id(), LinearSystemSolver::id());
  }

  // Now add an inconsistent constraint
  for (int i = 0; i < N; ++i) {
    auto x = prog_list.at(i).decision_variables();
    prog_list.at(i).AddLinearEqualityConstraint(x(0) + x(1), 5);
  }
  // Don't terminate early
  result_list = SolveInParallel(prog_list, 3, false);
  for (int i = 0; i < N; ++i) {
    EXPECT_FALSE(result_list.at(i)->is_success());
    EXPECT_EQ(result_list.at(i)->get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
    EXPECT_EQ(result_list.at(i)->get_solver_id(), LinearSystemSolver::id());
  }

  // Terminate early
  result_list = SolveInParallel(prog_list, 2, true);
  bool some_result_has_no_value = false;
  for (int i = 0; i < N; ++i) {
    if (result_list.at(i).has_value()) {
      EXPECT_FALSE(result_list.at(i)->is_success());
      EXPECT_EQ(result_list.at(i)->get_optimal_cost(),
                MathematicalProgram::kGlobalInfeasibleCost);
      EXPECT_EQ(result_list.at(i)->get_solver_id(), LinearSystemSolver::id());
    } else {
      some_result_has_no_value = true;
    }
  }
  EXPECT_TRUE(some_result_has_no_value);
}

GTEST_TEST(SolveTest, TestInitialGuessAndOptions) {
  // Test with gurobi solver, which accepts both initial guess and solver
  // options.
  MathematicalProgram prog;
  auto x = prog.NewBinaryVariables<1>();
  auto y = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(y(0) + y(1) == 1);
  if (GurobiSolver::is_available()) {
    ASSERT_TRUE(ChooseBestSolver(prog) == GurobiSolver::id());
    SolverOptions solver_options;
    // Presolve and Heuristics would each independently solve
    // this problem inside of the Gurobi solver, but without
    // consulting the initial guess.
    solver_options.SetOption(GurobiSolver::id(), "Presolve", 0);
    solver_options.SetOption(GurobiSolver::id(), "Heuristics", 0.0);
    Eigen::VectorXd vars_init(3);
    vars_init << 1, 0, 1;
    MathematicalProgramResult result = Solve(prog, vars_init, solver_options);
    EXPECT_NEAR(result.GetSolution(x)(0), vars_init(0), 1E-6);
  }
}

GTEST_TEST(SolveTest, ParallelTestInitialGuessAndOptions) {
  // Test with gurobi solver, which accepts both initial guess and solver
  // options.
  const int num_gurobi_prog = 10;
  const int num_snopt_prog = 10;
  const int N = num_gurobi_prog + num_snopt_prog;
  std::vector<MathematicalProgram> prog_list(N);
  for (int i = 0; i < static_cast<int>(prog_list.size()); ++i) {
    if (i < num_gurobi_prog) {
      // A MIP which will be solved by the gurobi solver, which accepts both
      // initial guess and solver options.
      auto x = prog_list.at(i).NewBinaryVariables<1>();
      auto y = prog_list.at(i).NewContinuousVariables<2>();
      prog_list.at(i).AddLinearConstraint(y(0) + y(1) == 1);
    } else {
      // An indefinite QP which will be solved by the snopt solver.
      auto x = prog_list.at(i).NewContinuousVariables<2>();
      prog_list.at(i).AddCost(x(0) * x(1));  // An indefinite quadratic cost
      prog_list.at(i).AddConstraint(pow(x(0), 2) + pow(x(1), 2) == 1);
    }
  }
  std::vector<std::optional<Eigen::VectorXd>> initial_guesses(prog_list.size());
  Eigen::VectorXd vars_init_gurobi(3);
  vars_init_gurobi << 1, 0, 1;
  Eigen::VectorXd vars_init_snopt(2);
  vars_init_snopt << 1, 2;
  if (GurobiSolver::is_available() && SnoptSolver::is_available()) {
    SolverOptions solver_options;
    // Presolve and Heuristics would each independently solve
    // this problem inside of the Gurobi solver, but without
    // consulting the initial guess.
    solver_options.SetOption(GurobiSolver::id(), "Presolve", 0);
    solver_options.SetOption(GurobiSolver::id(), "Heuristics", 0.0);
    for (int i = 0; i < static_cast<int>(prog_list.size()); ++i) {
      if (i < num_gurobi_prog) {
        ASSERT_TRUE(ChooseBestSolver(prog_list.at(i)) == GurobiSolver::id());
        if (i % 2 == 0) {
          // Set the initial guess for some but not all the gurobi problems.
          initial_guesses.at(i) = vars_init_gurobi;
        }
      } else {
        initial_guesses.at(i) = vars_init_snopt;
        ASSERT_TRUE(ChooseBestSolver(prog_list.at(i)) == SnoptSolver::id());
      }
    }
    auto result_list =
        SolveInParallel(prog_list, initial_guesses, solver_options, -1, false);
    for (int i = 0; i < static_cast<int>(prog_list.size()); ++i) {
      EXPECT_TRUE(result_list.at(i)->is_success());
      if (i < num_gurobi_prog) {
        if (i % 2 == 0) {
          // If the initial guess is set, then the optimal solution will have
          // the binary set to 1.
          EXPECT_NEAR(result_list.at(i)->get_x_val()(0), vars_init_gurobi(0),
                      1E-6);
        } else {
          // If the initial guess is not set, then the optimal solution will
          // have the binary set to 0.
          EXPECT_NEAR(result_list.at(i)->get_x_val()(0), 0, 1E-6);
        }
      } else {
        const double x = result_list.at(i)->get_x_val()(0);
        const double y = result_list.at(i)->get_x_val()(1);
        EXPECT_NEAR(std::pow(x, 2) + std::pow(y, 2), 1, 1E-6);
        EXPECT_NEAR(std::abs(x), 1 / std::sqrt(2), 1E-6);
        EXPECT_NEAR(std::abs(y), 1 / std::sqrt(2), 1E-6);
      }
    }
  }
}

}  // namespace solvers
}  // namespace drake
