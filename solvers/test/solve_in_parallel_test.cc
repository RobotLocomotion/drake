#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
namespace {

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
  // Cross-check that our BUILD file allows parallelism.
  DRAKE_DEMAND(Parallelism::Max().num_threads() > 1);

  // Test with gurobi solver, which uses an initial guess.
  MathematicalProgram prog;
  std::vector<const MathematicalProgram*> progs;
  std::vector<const Eigen::VectorXd*> initial_guesses;
  SolverOptions solver_options;
  std::optional<SolverId> solver_ids;

  // The initial guess
  Eigen::VectorXd initial_guess_0(3);
  initial_guess_0 << 1, 0, 1;
  Eigen::VectorXd initial_guess_1(3);
  initial_guess_1 << 0, 0, 1;
  // The solver options
  solver_options.SetOption(GurobiSolver::id(), "Presolve", 0);
  solver_options.SetOption(GurobiSolver::id(), "Heuristics", 0.0);

  // The variables used in the programs
  symbolic::Variable x("x", symbolic::Variable::Type::BINARY);
  symbolic::Variable y("y");
  symbolic::Variable z("z");
  Eigen::VectorX<symbolic::Variable> vars(3);
  vars << x, y, z;

  prog.AddDecisionVariables(vars);
  prog.AddLinearConstraint(y + z == 1);
  const int num_trials = 10;
  for (int i = 0; i < num_trials; i++) {
    progs.push_back(&prog);
    if (i < num_trials / 2) {
      initial_guesses.push_back(&initial_guess_0);
    } else {
      initial_guesses.push_back(&initial_guess_1);
    }
  }
  if (GurobiSolver::is_available()) {
    const std::vector<MathematicalProgramResult> results =
        SolveInParallel(progs, &initial_guesses, &solver_options,
                        GurobiSolver::id(), Parallelism::Max(), false);

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
  const int num_trials = 10;

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
        // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/statuscodes.html
        const int ITERATION_LIMIT = 7;
        EXPECT_EQ(solver_details.optimization_status, ITERATION_LIMIT);
        EXPECT_TRUE(std::isfinite(results.at(i).get_optimal_cost()));
        EXPECT_TRUE(results.at(i).GetSolution(x).array().isFinite().all());
      } else {
        // The last num_trials/2 programs should solve to optimality.
        // This code is defined in
        // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/statuscodes.html
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
        progs, nullptr, &solver_options_0, solver_id, Parallelism::Max(), true);

    // All programs should hit the iteration limit.
    for (int i = 0; i < num_trials; i++) {
      const auto solver_details =
          results.at(i).get_solver_details<GurobiSolver>();
      // This code is defined in
      // https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/statuscodes.html
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
  auto constraint4 =
      non_convex_prog.AddQuadraticConstraint(y(0) * y(0) + y(1) * y(1), 0, 1);
  const SolverId non_convex_solver = IpoptSolver::id();

  std::unique_ptr<MathematicalProgram> non_convex_non_threadsafe_prog =
      non_convex_prog.Clone();
  // Adds an expression cost that has no effect on the solution of the
  // program, but makes the program non-threadsafe.
  non_convex_non_threadsafe_prog->AddCost(cos(y(0)));

  // Ensure that the programs meet the expected thread-safety. This ensures that
  // if certain costs/constraints are flagged for different thread safety in the
  // future, this test will fail and need to be updated.
  ASSERT_TRUE(convex_prog.IsThreadSafe());
  ASSERT_TRUE(non_convex_prog.IsThreadSafe());
  ASSERT_FALSE(non_convex_non_threadsafe_prog->IsThreadSafe());

  // Solve in parallel.
  std::vector<const MathematicalProgram*> progs;
  std::vector<std::optional<SolverId>> solver_ids;
  const int num_trials = 10;
  for (int i = 0; i < num_trials; i++) {
    if (i % 3 == 0) {
      progs.push_back(&convex_prog);
      solver_ids.push_back(convex_solver);
    } else if (i % 3 == 1) {
      progs.push_back(&non_convex_prog);
      solver_ids.push_back(non_convex_solver);
    } else if (i % 3 == 2) {
      progs.push_back(non_convex_non_threadsafe_prog.get());
      solver_ids.push_back(non_convex_solver);
    }
  }
  std::vector<MathematicalProgramResult> results = SolveInParallel(
      progs, nullptr, nullptr, &solver_ids, Parallelism::Max(), false);
  for (int i = 0; i < num_trials; i++) {
    if (i % 3 == 0) {
      EXPECT_EQ(results.at(i).get_solver_id(), convex_solver);
    } else {
      // Both the threadsafe and non-threadsafe nonconvex program use the same
      // solver.
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
  EXPECT_THROW(SolveInParallel(progs, nullptr, nullptr, convex_solver,
                               Parallelism::Max(), false),
               std::exception);

  // We expect the overload to run if a non-convex solver is passed as the
  // solver, even if it is not the best choice.
  results = SolveInParallel(progs, nullptr, nullptr, non_convex_solver,
                            Parallelism::Max(), false);
  for (int i = 0; i < num_trials; i++) {
    EXPECT_EQ(results.at(i).get_solver_id(), non_convex_solver);
  }
}

/* Test fixture that runs SolveInParallel against a specific solver_id. When run
with Drake CI's sanitizers and memory checkers, this would flag memory or thread
errors seen during the parallel solves. */
class SolveInParallelIntegrationTest
    : public ::testing::TestWithParam<Parallelism> {
 public:
  std::vector<MathematicalProgramResult> Run(const SolverId& solver_id,
                                             const MathematicalProgram& prog) {
    // Bail out in case the solver was disabled in this particular build.
    auto solver = MakeSolver(solver_id);
    if (!solver->available()) {
      return {};
    }
    // Don't hold onto any extra resources during SolveInParallel.
    solver.reset();
    // Solve the same program repeatedly, with max parallelism (as configured in
    // our BUILD file).
    Parallelism parallelism = GetParam();
    // Don't solve too many programs if we are not able to parallelize the
    // solve.
    const size_t num_programs = parallelism.num_threads() > 1 ? 100 : 10;
    std::vector<const MathematicalProgram*> progs(num_programs, &prog);
    return SolveInParallel(progs, {}, {}, solver_id, Parallelism::Max());
  }
};

TEST_P(SolveInParallelIntegrationTest, Clarabel) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(ClarabelSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Clp) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(ClpSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Csdp) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(CsdpSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Gurobi) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(GurobiSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Ipopt) {
  QuadraticProgram1 qp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(IpoptSolver::id(), *qp.prog())) {
    qp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Mosek) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(MosekSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Nlopt) {
  NonConvexQPproblem1 qp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  // Set an initial guess to avoid the suboptimal, stationary point 0.
  qp.prog()->SetInitialGuessForAllVariables(
      (Eigen::VectorXd(5) << 1.5, 1.5, -1.5, 1.5, -1.5).finished());
  for (const auto& result : Run(NloptSolver::id(), *qp.prog())) {
    qp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Osqp) {
  QuadraticProgram1 qp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(OsqpSolver::id(), *qp.prog())) {
    qp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Scs) {
  LinearProgram2 lp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(ScsSolver::id(), *lp.prog())) {
    lp.CheckSolution(result);
  }
}

TEST_P(SolveInParallelIntegrationTest, Snopt) {
  QuadraticProgram1 qp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  for (const auto& result : Run(SnoptSolver::id(), *qp.prog())) {
    qp.CheckSolution(result);
  }
}
// Test SolveInParallel with and without parallelism enabled
INSTANTIATE_TEST_SUITE_P(test, SolveInParallelIntegrationTest,
                         ::testing::Values(Parallelism::Max(),
                                           Parallelism::None()));  // NOLINT

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
