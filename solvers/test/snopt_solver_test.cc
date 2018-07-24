#include "drake/solvers/snopt_solver.h"

#include <iostream>
#include <thread>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/quadratic_program_examples.h"

namespace drake {
namespace solvers {
namespace test {

TEST_P(LinearProgramTest, TestLP) {
  SnoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    SnoptTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestSnopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d(1, 2));
  SnoptSolver solver;
  if (solver.available()) {
    const auto solver_result = solver.Solve(*prog_);
    EXPECT_EQ(solver_result, SolutionResult::kInfeasibleConstraints);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestSnopt) {
  prog_->SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  SnoptSolver solver;
  if (solver.available()) {
    const auto solver_result = solver.Solve(*prog_);
    EXPECT_EQ(solver_result, SolutionResult::kUnbounded);
    EXPECT_EQ(prog_->GetOptimalCost(),
              -std::numeric_limits<double>::infinity());
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  SnoptSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
    SnoptTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  SnoptSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(SnoptTest, TestSetOption) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  // Solve a program
  // min x(0) + x(1) + x(2)
  // s.t xᵀx=1
  prog.AddLinearCost(x.cast<symbolic::Expression>().sum());
  prog.AddConstraint(
      std::make_shared<QuadraticConstraint>(2 * Eigen::Matrix3d::Identity(),
                                            Eigen::Vector3d::Zero(), 1, 1),
      x);

  // Arbitrary initial guess.
  prog.SetInitialGuess(x, Eigen::Vector3d(10, 20, 30));

  SnoptSolver solver;
  // Make sure the default setting can solve the problem.
  SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);

  // The program is infeasible after one major iteration.
  prog.SetSolverOption(SnoptSolver::id(), "Major iterations limit", 1);
  result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kIterationLimit);
}

GTEST_TEST(SnoptTest, TestPrintFile) {
// SNOPT 7.6 has a known bug where it mis-handles the NIL terminator byte
// when setting an debug log filename.  This is fixed in newer releases.
// Ideally we would add this function to the asan blacklist, but SNOPT
// doesn't have symbols so we'll just disable this test case instead.
#if __has_feature(address_sanitizer) or defined(__SANITIZE_ADDRESS__)
  constexpr bool using_asan = true;
#else
  constexpr bool using_asan = false;
#endif
  if (using_asan) {
    std::cerr << "Skipping TestPrintFile under ASAN\n";
    return;
  }

  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) == 1);

  // This is to verify we can set the print out file.
  const std::string print_file = temp_directory() + "/snopt.out";
  std::cout << print_file << std::endl;
  EXPECT_FALSE(spruce::path(print_file).exists());
  prog.SetSolverOption(SnoptSolver::id(), "Print file", print_file);
  const SnoptSolver solver;
  const SolutionResult result = solver.Solve(prog);
  EXPECT_EQ(result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(spruce::path(print_file).exists());
}

GTEST_TEST(SnoptTest, TestSparseCost) {
  // Test nonlinear optimization problem, whose cost has sparse gradient.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  // No cost, just constraint.
  prog.AddLinearConstraint(x(0) + x(1) + x(2) == 1);

  SnoptSolver solver;
  auto solution_result = solver.Solve(prog);
  EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
  const double tol{1E-5};
  EXPECT_NEAR(prog.GetSolution(x).sum(), 1, tol);
  EXPECT_EQ(prog.GetOptimalCost(), 0);

  // Add a cost on x(1) only
  // min x(1) * x(1).
  // s.t x(0) + x(1) + x(2) = 1
  prog.AddQuadraticCost(x(1) * x(1));
  solution_result = solver.Solve(prog);
  EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(x(1)), 0, tol);
  EXPECT_NEAR(prog.GetSolution(x).sum(), 1, tol);
  EXPECT_NEAR(prog.GetOptimalCost(), 0, tol);

  // Add another cost on x(1) and x(2)
  // min x(1) * x(1) + 2 * x(1) + x(2) * x(2) - 2 * x(2) + 1
  // s.t x(0) + x(1) + x(2) = 1
  prog.AddQuadraticCost(2 * x(1) + x(2) * x(2) - 2 * x(2) + 1);
  solution_result = solver.Solve(prog);
  EXPECT_EQ(solution_result, SolutionResult::kSolutionFound);
  EXPECT_TRUE(
      CompareMatrices(prog.GetSolution(x), Eigen::Vector3d(1, -1, 1), tol));
  EXPECT_NEAR(prog.GetOptimalCost(), -1, tol);
}

GTEST_TEST(SnoptTest, MultiThreadTest) {
  // Test if we can run several snopt solvers simultaneously on multiple
  // threads.
  // We create a convex QP problem with a unique global optimal, starting from
  // different initial guesses, snopt should output the same result.
  // min (x₀-1)² + (x₁-2)²
  // s.t x₀ + x₁ = 1
  // The optimal solution is x*=(0, 1)
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  const Eigen::Vector2d c(1, 2);
  prog.AddQuadraticCost((x - c).squaredNorm());

  prog.AddLinearConstraint(x(0) + x(1) == 1);

  SnoptSolver snopt_solver;

  auto SolveWithInitialGuess = [&snopt_solver, &prog](
      const Eigen::Vector2d& x_init, const std::string& print_file_name) {
    int snopt_status;
    double objective;
    Eigen::Vector2d x_val;

    std::map<std::string, std::string> snopt_options_string =
        prog.GetSolverOptionsStr(snopt_solver.solver_id());
    // Set the print file
    const std::string print_file = temp_directory() + print_file_name;
    std::cout << print_file << std::endl;
    EXPECT_FALSE(spruce::path(print_file).exists());
    snopt_options_string["Print file"] = print_file;

    snopt_solver.DoSolve(prog, x_init, snopt_options_string,
                         prog.GetSolverOptionsInt(snopt_solver.solver_id()),
                         prog.GetSolverOptionsDouble(snopt_solver.solver_id()),
                         &snopt_status, &objective, &x_val);
    EXPECT_EQ(snopt_status, 1);
    EXPECT_NEAR(objective, 2, 1E-6);
    EXPECT_TRUE(CompareMatrices(x_val, Eigen::Vector2d(0, 1), 1E-6));
    EXPECT_TRUE(spruce::path(print_file).exists());
  };

  const int num_threads = 10;
  std::vector<std::thread> test_threads;
  for (int i = 0; i < num_threads; ++i) {
    // Arbitrary initial guess.
    const Eigen::Vector2d x_init(i + 1, (i - 2.0) / 10);
    const std::string print_file_name =
        "snopt_multithread" + std::to_string(i) + ".out";
    test_threads.emplace_back(SolveWithInitialGuess, x_init, print_file_name);
  }
  for (int i = 0; i < num_threads; ++i) {
    test_threads[i].join();
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
