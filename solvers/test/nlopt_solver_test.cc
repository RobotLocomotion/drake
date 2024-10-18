#include "drake/solvers/nlopt_solver.h"

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
TEST_F(UnboundedLinearProgramTest0, TestNlopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  NloptSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, {}, &result);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_optimal_cost(),
              -std::numeric_limits<double>::infinity());
    SolverOptions solver_options;
    solver_options.SetOption(solver.solver_id(), NloptSolver::MaxEvalName(), 1);
    solver.Solve(*prog_, {}, solver_options, &result);
    const int NLOPT_MAXEVAL_REACHED = 5;
    EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
              NLOPT_MAXEVAL_REACHED);
  }
}

GTEST_TEST(QPtest, TestUnitBallExample) {
  NloptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(NloptSolverTest, TestNonconvexQP) {
  NloptSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, false);
  }
}

GTEST_TEST(NloptSolverTest, SetAlgorithm) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Ones(), x,
                        true /*is_convex */);
  prog.AddBoundingBoxConstraint(1, 2, x);
  NloptSolver solver;
  SolverOptions solver_options;
  solver_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                           "LD_CCSAQ");
  const auto result =
      solver.Solve(prog, Eigen::VectorXd::Ones(2), solver_options);
  ASSERT_TRUE(result.is_success());

  solver_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                           "FOO_BAR");
  DRAKE_EXPECT_THROWS_MESSAGE(
      solver.Solve(prog, Eigen::VectorXd::Ones(2), solver_options),
      ".*Unknown.*algorithm.*");
}

TEST_F(QuadraticEqualityConstrainedProgram1, Test) {
  NloptSolver solver;
  if (solver.is_available()) {
    CheckSolution(solver, Eigen::Vector2d(0.5, 0.8), std::nullopt, 1E-4,
                  false /* check dual */);
  }
}

// This test checks that calling NloptSolver in parallel does not cause any
// threading issues.
GTEST_TEST(NloptTest, TestSolveInParallel) {
  int num_problems = 100;
  NonConvexQPproblem1 qp{CostForm::kNonSymbolic, ConstraintForm::kNonSymbolic};
  std::vector<const MathematicalProgram*> progs;
  // Give the program an initial guess to avoid the suboptimal, stationary point
  // 0.
  Eigen::VectorXd initial_guess{5};
  initial_guess << 1.5, 1.5, -1.5, 1.5, -1.5;
  std::vector<const Eigen::VectorXd*> initial_guesses;
  for (int i = 0; i < num_problems; ++i) {
    progs.push_back(qp.prog());
    initial_guesses.push_back(&initial_guess);
  }

  std::vector<MathematicalProgramResult> results = SolveInParallel(
      progs, &initial_guesses, std::nullopt /* no solver options */,
      NloptSolver::id(), Parallelism::Max());
  for (int i = 0; i < num_problems; ++i) {
    qp.CheckSolution(results[i]);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
