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

GTEST_TEST(SolveInParallelTest, OverloadIsNotAmbiguousTest) {
  // This test checks that the overloads of SolveInParallel are not ambiguous.
  const int num_trials = 2;
  std::vector<std::unique_ptr<MathematicalProgram>> progs(num_trials);
  std::vector<const MathematicalProgram*> prog_ptrs;
  for (int i = 0; i < num_trials; i++) {
    progs.push_back(std::make_unique<MathematicalProgram>());
    prog_ptrs.push_back(progs.back().get());
  }
  SolveInParallel(prog_ptrs);
}

GTEST_TEST(SolveInParallelTest, TestSolveInParallelInitialGuess) {
  // Test with gurobi solver, which uses an initial guess.
  std::vector<std::unique_ptr<MathematicalProgram>> progs;
  std::vector<const MathematicalProgram*> prog_ptrs;
  std::vector<const Eigen::VectorXd*> initial_guesses;
  std::vector<const SolverOptions*> solver_options;
  std::vector<std::optional<SolverId>> solver_ids;

  // The initial guess
  Eigen::VectorXd initial_guess_0(3);
  initial_guess_0 << 1, 0, 1;
  Eigen::VectorXd initial_guess_1(3);
  initial_guess_1 << 0, 0, 1;
  // The solver options
  SolverOptions solver_options_local;
  solver_options_local.SetOption(GurobiSolver::id(), "Presolve", 0);
  solver_options_local.SetOption(GurobiSolver::id(), "Heuristics", 0.0);

  // The variables used in the programs
  symbolic::Variable x("x", symbolic::Variable::Type::BINARY);
  symbolic::Variable y("y");
  symbolic::Variable z("y");
  Eigen::VectorX<symbolic::Variable> vars(3);
  vars << x, y, z;

  int num_trials = 10;
  for (int i = 0; i < num_trials; i++) {
    progs.push_back(std::make_unique<MathematicalProgram>());
    progs.back()->AddDecisionVariables(vars);
    progs.back()->AddLinearConstraint(y + z == 1);
    prog_ptrs.push_back(progs.back().get());
    if (i < num_trials / 2) {
      initial_guesses.push_back(&initial_guess_0);
    } else {
      initial_guesses.push_back(&initial_guess_1);
    }
    solver_options.push_back(&solver_options_local);
    solver_ids.push_back(GurobiSolver::id());
  }
  if (GurobiSolver::is_available()) {
    const std::vector<MathematicalProgramResult> results =
        SolveInParallel(prog_ptrs, &initial_guesses, &solver_options,
                        &solver_ids, Parallelism::Max(), false);

    for (int i = 0; i < num_trials; i++) {
      if (i < num_trials / 2) {
        // We used an initial guess and disabled the presolve and heuristics
        // therefore we expect the solution to be the initial guess.
        EXPECT_NEAR(results.at(i).GetSolution(x), initial_guess_0(0), 1E-6);
      } else {
        // We did not use an initial guess and disabled the presolve and
        // heuristics. Gurobi's heuristics set x to 1.
        EXPECT_NEAR(results.at(i).GetSolution(x), initial_guess_1(0), 1E-6);
      }
    }
  }
}

GTEST_TEST(SolveInParallelTest, TestSolveInParallelSolverOptions) {
  MathematicalProgram prog;
  std::vector<const MathematicalProgram*> progs;
  std::vector<const SolverOptions*> solver_options;
  std::vector<std::optional<SolverId>> solver_ids;
  int num_trials = 10;

  // This is a program which would require more than one iteration to solve.
  auto x = prog.NewContinuousVariables<4>();
  prog.AddLorentzConeConstraint(x);
  auto constraint2 = prog.AddLinearConstraint(x(0) + x(1) + x(2) + x(3) == 2);
  prog.AddRotatedLorentzConeConstraint(x);
  prog.AddLinearCost(x(0) + 2 * x(1));

  SolverId solver_id = GurobiSolver::id();

  SolverOptions solver_options_0;
  solver_options_0.SetOption(solver_id, "IterationLimit", 1);
  solver_options_0.SetOption(solver_id, "BarIterLimit", 1);
  solver_options_0.SetOption(solver_id, "QCPDual", 1);

  for (int i = 0; i < num_trials; i++) {
    progs.push_back(&prog);

    if (i < num_trials / 2) {
      solver_options.push_back(&solver_options_0);
    } else {
      solver_options.push_back(nullptr);
    }
    solver_ids.push_back(GurobiSolver::id());
  }

  if (GurobiSolver::is_available()) {
    const std::vector<MathematicalProgramResult> results = SolveInParallel(
        progs, nullptr, &solver_options, &solver_ids, Parallelism::Max(), true);

    for (int i = 0; i < num_trials; i++) {
      const auto solver_details =
          results.at(i).get_solver_details<GurobiSolver>();
      if (i < num_trials / 2) {
        // The first num_trials/2 programs should hit the iteration limit.
        // This code is defined in
        // https://www.gurobi.com/documentation/10.0/refman/optimization_status_codes.html
        const int ITERATION_LIMIT = 7;
        EXPECT_EQ(solver_details.optimization_status, ITERATION_LIMIT);
        EXPECT_TRUE(std::isfinite(results.at(i).get_optimal_cost()));
        EXPECT_TRUE(results.at(i).GetSolution(x).array().isFinite().all());
      } else {
        // The last num_trials/2 programs should solve to optimality.
        // This code is defined in
        // https://www.gurobi.com/documentation/10.0/refman/optimization_status_codes.html
        const int OPTIMAL = 2;
        EXPECT_EQ(solver_details.optimization_status, OPTIMAL);
        EXPECT_TRUE(std::isfinite(results.at(i).get_optimal_cost()));
        EXPECT_TRUE(results.at(i).GetSolution(x).array().isFinite().all());
      }
    }
  }
  // Now test the overload.
  if (GurobiSolver::is_available()) {
    const std::vector<MathematicalProgramResult> results = SolveInParallel(
        progs, nullptr, solver_options_0, solver_id, Parallelism::Max(), true);

    for (int i = 0; i < num_trials; i++) {
      const auto solver_details =
          results.at(i).get_solver_details<GurobiSolver>();
      // The first num_trials/2 programs should hit the iteration limit.
      // This code is defined in
      // https://www.gurobi.com/documentation/10.0/refman/optimization_status_codes.html
      const int ITERATION_LIMIT = 7;
      EXPECT_EQ(solver_details.optimization_status, ITERATION_LIMIT);
      EXPECT_TRUE(std::isfinite(results.at(i).get_optimal_cost()));
      EXPECT_TRUE(results.at(i).GetSolution(x).array().isFinite().all());
    }
  }
}

GTEST_TEST(SolveInParallel, TestDiversePrograms) {
  // A Convex Program
  MathematicalProgram convex_prog;
  auto x = convex_prog.NewContinuousVariables<4>();
  convex_prog.AddLorentzConeConstraint(x);
  auto constraint2 =
      convex_prog.AddLinearConstraint(x(0) + x(1) + x(2) + x(3) == 2);
  convex_prog.AddRotatedLorentzConeConstraint(x);
  convex_prog.AddLinearCost(x(0) + 2 * x(1));
  const SolverId convex_solver = ScsSolver::id();

  MathematicalProgram non_convex_prog;
  auto y = non_convex_prog.NewContinuousVariables<2>();
  // An indefinite quadratic cost.
  auto cost = non_convex_prog.AddQuadraticCost(y(0) * y(0) - y(1) * y(1));
  auto constraint3 = non_convex_prog.AddLinearConstraint(y(0) + y(1) == 1);
  const SolverId non_convex_solver = SnoptSolver::id();

  std::vector<const MathematicalProgram*> progs;
  std::vector<std::optional<SolverId>> solver_ids;
  int num_trials = 10;
  for (int i = 0; i < num_trials; i++) {
    if (i % 2 == 0) {
      progs.push_back(&convex_prog);
      solver_ids.push_back(convex_solver);
    } else {
      progs.push_back(&non_convex_prog);
      solver_ids.push_back(non_convex_solver);
    }
  }
  std::vector<MathematicalProgramResult> results = SolveInParallel(
      progs, nullptr, nullptr, &solver_ids, Parallelism::Max(), false);
  for (int i = 0; i < num_trials; i++) {
    if (i % 2 == 0) {
      EXPECT_EQ(results.at(i).get_solver_id(), convex_solver);
    } else {
      EXPECT_EQ(results.at(i).get_solver_id(), non_convex_solver);
    }
  }

  // Now test when the solvers are not specified.
  results = SolveInParallel(progs, nullptr, nullptr, nullptr,
                            Parallelism::Max(), false);
  for (int i = 0; i < num_trials; i++) {
    EXPECT_EQ(results.at(i).get_solver_id(), ChooseBestSolver(*progs.at(i)));
  }

  // We expect the overload to fail if only a convex solver is passed.
  EXPECT_THROW(SolveInParallel(progs, nullptr, std::nullopt, convex_solver,
                               Parallelism::Max(), false),
               std::exception);

  // We expect the overload to run if a non-convex solver is passed as the
  // solver, even if it is not the best choice.
  results = SolveInParallel(progs, nullptr, std::nullopt, non_convex_solver,
                            Parallelism::Max(), false);
  for (int i = 0; i < num_trials; i++) {
    EXPECT_EQ(results.at(i).get_solver_id(), non_convex_solver);
  }
}

}  // namespace solvers
}  // namespace drake
