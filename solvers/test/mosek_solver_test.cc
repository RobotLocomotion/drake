#include "drake/solvers/mosek_solver.h"

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/temp_directory.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/test/semidefinite_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
TEST_P(LinearProgramTest, TestLP) {
  MosekSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(UnboundedLinearProgramTest0, Test) {
  MosekSolver solver;
  if (solver.available()) {
    const MathematicalProgram& const_prog = *prog_;
    MathematicalProgramResult result;
    solver.Solve(const_prog, {}, {}, &result);
    // Mosek can only detect dual infeasibility, not primal unboundedness.
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
    EXPECT_EQ(
        result.get_solver_details().GetValue<MosekSolverDetails>().rescode, 0);
  }
}

TEST_F(UnboundedLinearProgramTest1, Test) {
  MosekSolver solver;
  if (solver.available()) {
    const SolutionResult result = solver.Solve(*prog_);
    // Mosek can only detect dual infeasibility, not primal unboundedness.
    EXPECT_EQ(result, SolutionResult::kDualInfeasible);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  MosekSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  MosekSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(MosekTest, TestEllipsoidsSeparation,
                        ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(MosekTest, TestQPasSOCP,
                        ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveAndCheckSolution(mosek_solver);
  }
}

INSTANTIATE_TEST_CASE_P(
    MosekTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSemidefiniteProgram, TrivialSDP) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    TestTrivialSDP(mosek_solver, 1E-8);
  }
}

GTEST_TEST(TestSemidefiniteProgram, CommonLyapunov) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    FindCommonLyapunov(mosek_solver, 1E-8);
  }
}

GTEST_TEST(TestSemidefiniteProgram, OuterEllipsoid) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    FindOuterEllipsoid(mosek_solver, 1E-6);
  }
}

GTEST_TEST(TestSemidefiniteProgram, EigenvalueProblem) {
  MosekSolver mosek_solver;
  if (mosek_solver.available()) {
    SolveEigenvalueProblem(mosek_solver, 1E-7);
  }
}

GTEST_TEST(MosekTest, TestLogFile) {
  // Test if we can print the logging info to a log file.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) == 1);

  const std::string log_file = temp_directory() + "/mosek.log";
  EXPECT_FALSE(spruce::path(log_file).exists());
  MosekSolver solver;
  solver.Solve(prog);
  // By default, no logging file.
  EXPECT_FALSE(spruce::path(log_file).exists());
  // Output the logging to the console
  solver.set_stream_logging(true, "");
  solver.Solve(prog);
  EXPECT_FALSE(spruce::path(log_file).exists());
  // Output the logging to the file.
  solver.set_stream_logging(true, log_file);
  solver.Solve(prog);
  EXPECT_TRUE(spruce::path(log_file).exists());
}

GTEST_TEST(MosekTest, SolverOptionsTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) >= 0);
  prog.AddLinearConstraint(x(1) <= 1);
  prog.AddLinearConstraint(x(0) <= 1);
  // The Hessian of the quadratic cost has an eigen value -1e-8, so it is not
  // positive semidefinite. By adjusting Mosek's tolerance on the semidefinite
  // matrix, we expect the solver to give a different result.
  prog.AddQuadraticCost(x(0) * x(0) - 1E-8 * x(1) * x(1));

  SolverOptions solver_options;
  // First try a big positive semidefinite matrix tolerance.
  solver_options.SetOption(MosekSolver::id(),
                           "MSK_DPAR_SEMIDEFINITE_TOL_APPROX", 1E-6);
  MathematicalProgramResult result;
  MosekSolver mosek_solver;
  mosek_solver.Solve(prog, {}, solver_options, &result);
  EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);

  // Now try a small positive semidefinite matrix tolerance.
  solver_options.SetOption(MosekSolver::id(),
                           "MSK_DPAR_SEMIDEFINITE_TOL_APPROX", 1E-10);
  mosek_solver.Solve(prog, {}, solver_options, &result);
  EXPECT_NE(result.get_solution_result(), SolutionResult::kSolutionFound);
}

GTEST_TEST(MosekSolver, SolverOptionsErrorTest) {
  // Set a non-existing option. Mosek should report error.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) >= 0);

  MathematicalProgramResult result;
  MosekSolver mosek_solver;
  SolverOptions solver_options;
  solver_options.SetOption(MosekSolver::id(), "non-existing options", 42);
  mosek_solver.Solve(prog, {}, solver_options, &result);
  const MosekSolverDetails solver_details =
      result.get_solver_details().GetValue<MosekSolverDetails>();
  // This response code is defined in
  // https://docs.mosek.com/8.1/capi/response-codes.html#mosek.rescode
  const int MSK_RES_ERR_PARAM_NAME_INT = 1207;
  EXPECT_EQ(solver_details.rescode, MSK_RES_ERR_PARAM_NAME_INT);
  // This problem status is defined in
  // https://docs.mosek.com/8.1/capi/constants.html#mosek.prosta
  const int MSK_PRO_STA_UNKNOWN = 0;
  EXPECT_EQ(solver_details.solution_status, MSK_PRO_STA_UNKNOWN);

  EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnknownError);
}
}  // namespace test
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
