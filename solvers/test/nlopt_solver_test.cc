#include "drake/solvers/nlopt_solver.h"

#include <gtest/gtest.h>

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

GTEST_TEST(NloptSolverTest, TestL2NormCost) {
  NloptSolver solver;
  TestL2NormCost(solver, 1e-6);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
