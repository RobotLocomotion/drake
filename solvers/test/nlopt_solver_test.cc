#include "drake/solvers/nlopt_solver.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
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

    const int NLOPT_MAXEVAL_REACHED = 5;
    SolverOptions solver_options1;
    solver_options1.SetOption(solver.solver_id(), NloptSolver::MaxEvalName(),
                              1);
    solver.Solve(*prog_, {}, solver_options1, &result);
    EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
              NLOPT_MAXEVAL_REACHED);

    const int NLOPT_MAXTIME_REACHED = 6;
    SolverOptions solver_options2;
    solver_options2.SetOption(solver.solver_id(), NloptSolver::MaxTimeName(),
                              1e-10);
    solver.Solve(*prog_, {}, solver_options2, &result);
    EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
              NLOPT_MAXTIME_REACHED);

    // The cost is unbounded below, so any finite stopval is reached.
    const int NLOPT_STOPVAL_REACHED = 2;
    SolverOptions solver_options3;
    solver_options3.SetOption(solver.solver_id(), NloptSolver::StopValName(),
                              -100.0);
    solver.Solve(*prog_, {}, solver_options3, &result);
    EXPECT_EQ(result.get_solver_details<NloptSolver>().status,
              NLOPT_STOPVAL_REACHED);
    EXPECT_LE(result.get_optimal_cost(), -100.0);
    // The point NLopt stopped at is feasible here, so the feasibility
    // re-check leaves the result as a success.
    EXPECT_TRUE(
        prog_->CheckSatisfied(prog_->GetAllConstraints(), result.get_x_val()));
    EXPECT_TRUE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
  }
}

// Regression test for issue #24960: a constraint row with vacuous bounds
// (-inf, inf) should be ignored, not cause a solver error.
GTEST_TEST(NloptSolverTest, VacuousConstraintRow) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddCost(x.cast<symbolic::Expression>().dot(x));
  const double kInf = std::numeric_limits<double>::infinity();
  Eigen::Matrix<double, 2, 2> A;
  A << 1, 1, 1, -1;
  Eigen::Vector2d lb(1.0, -kInf);
  Eigen::Vector2d ub(1.0, kInf);
  prog.AddLinearConstraint(A, lb, ub, x);
  prog.SetInitialGuess(x, Eigen::Vector2d::Zero());
  NloptSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    EXPECT_TRUE(result.is_success());
    EXPECT_NEAR(result.GetSolution(x)(0) + result.GetSolution(x)(1), 1.0, 1e-6);
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

const Eigen::Vector2d kLocalOptimizerInitialGuess(0.5, 0.8);

// By default the f tolerances are disabled and NLopt stops on XTOL. Each f
// tolerance below is wide enough that the very first step satisfies it, so it
// must take over as the reason NLopt stops.
TEST_F(QuadraticEqualityConstrainedProgram1, FTolerances) {
  NloptSolver solver;
  if (solver.available()) {
    const int NLOPT_FTOL_REACHED = 3;
    const int NLOPT_XTOL_REACHED = 4;
    const auto baseline = solver.Solve(*prog_, kLocalOptimizerInitialGuess, {});
    ASSERT_TRUE(baseline.is_success());
    EXPECT_EQ(baseline.get_solver_details<NloptSolver>().status,
              NLOPT_XTOL_REACHED);

    SolverOptions ftol_rel_options;
    ftol_rel_options.SetOption(solver.id(),
                               NloptSolver::FRelativeToleranceName(), 1e3);
    const auto ftol_rel_result =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, ftol_rel_options);
    EXPECT_EQ(ftol_rel_result.get_solver_details<NloptSolver>().status,
              NLOPT_FTOL_REACHED);

    SolverOptions ftol_abs_options;
    ftol_abs_options.SetOption(solver.id(),
                               NloptSolver::FAbsoluteToleranceName(), 1e3);
    const auto ftol_abs_result =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, ftol_abs_options);
    EXPECT_EQ(ftol_abs_result.get_solver_details<NloptSolver>().status,
              NLOPT_FTOL_REACHED);
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerDefaults) {
  NloptSolver solver;
  if (solver.available()) {
    // With no local optimizer named, NLopt defaults it to LD_MMA with the
    // outer x tolerances and no evaluation cap, so naming LD_MMA explicitly
    // must change nothing.
    SolverOptions implicit_options;
    implicit_options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                               "LD_AUGLAG_EQ");
    const auto implicit_result =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, implicit_options);

    SolverOptions explicit_options = implicit_options;
    explicit_options.SetOption(
        solver.id(), NloptSolver::LocalOptimizerAlgorithmName(), "LD_MMA");
    const auto explicit_result =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, explicit_options);

    EXPECT_EQ(implicit_result.get_solution_result(),
              explicit_result.get_solution_result());
    EXPECT_TRUE(CompareMatrices(implicit_result.GetSolution(x_),
                                explicit_result.GetSolution(x_), 1e-12));
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerMaxEval) {
  NloptSolver solver;
  if (solver.available()) {
    SolverOptions options;
    options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                      "LD_AUGLAG_EQ");
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                      "LD_MMA");
    options.SetOption(solver.id(), NloptSolver::MaxEvalName(), 50);
    const auto uncapped =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

    // Truncating each subproblem to a single evaluation changes the iterates.
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerMaxEvalName(), 1);
    const auto capped =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

    EXPECT_FALSE(CompareMatrices(uncapped.GetSolution(x_),
                                 capped.GetSolution(x_), 1e-12));
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerFTolerance) {
  NloptSolver solver;
  if (solver.available()) {
    SolverOptions options;
    options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                      "LD_AUGLAG_EQ");
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                      "LD_MMA");
    options.SetOption(solver.id(), NloptSolver::MaxEvalName(), 50);
    const auto uncapped =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

    // A wide enough f tolerance stops each subproblem almost immediately, so
    // the outer algorithm follows a different path and lands elsewhere.
    options.SetOption(solver.id(),
                      NloptSolver::LocalOptimizerFAbsoluteToleranceName(), 1e3);
    const auto capped =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

    EXPECT_FALSE(CompareMatrices(uncapped.GetSolution(x_),
                                 capped.GetSolution(x_), 1e-12));
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerAlgorithm) {
  NloptSolver solver;
  // Drake's NLopt build omits src/algs/luksan/**, so the derivative-based
  // local algorithms are LD_MMA, LD_CCSAQ and LD_SLSQP.
  SolverOptions options;
  options.SetOption(solver.id(), NloptSolver::AlgorithmName(), "LD_AUGLAG_EQ");
  options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                    "LD_SLSQP");
  CheckSolution(solver, kLocalOptimizerInitialGuess, options, 1E-4,
                false /* check dual */);
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerUnknownAlgorithm) {
  NloptSolver solver;
  if (solver.available()) {
    SolverOptions options;
    options.SetOption(solver.id(), NloptSolver::AlgorithmName(),
                      "LD_AUGLAG_EQ");
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                      "FOO_BAR");
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options),
        ".*Unknown.*algorithm.*");
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, LocalOptimizerUnused) {
  NloptSolver solver;
  if (solver.available()) {
    // The default LD_SLSQP has no local optimizer, so the options are ignored.
    const auto plain = solver.Solve(*prog_, kLocalOptimizerInitialGuess, {});

    SolverOptions options;
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerAlgorithmName(),
                      "LD_SLSQP");
    options.SetOption(solver.id(), NloptSolver::LocalOptimizerMaxEvalName(), 3);
    options.SetOption(solver.id(),
                      NloptSolver::LocalOptimizerFRelativeToleranceName(), 1e3);
    options.SetOption(solver.id(),
                      NloptSolver::LocalOptimizerFAbsoluteToleranceName(), 1e3);
    const auto unused =
        solver.Solve(*prog_, kLocalOptimizerInitialGuess, options);

    ASSERT_TRUE(unused.is_success());
    EXPECT_TRUE(
        CompareMatrices(plain.GetSolution(x_), unused.GetSolution(x_), 1e-12));
  }
}

TEST_F(QuadraticEqualityConstrainedProgram1, Test) {
  NloptSolver solver;
  if (solver.is_available()) {
    CheckSolution(solver, Eigen::Vector2d(0.5, 0.8), std::nullopt, 1E-4,
                  false /* check dual */);
  }
}

GTEST_TEST(NloptSolverTest, TestL2NormCost) {
  NloptSolver solver;
  TestL2NormCost(solver, 1e-6);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
