#include "drake/solvers/osqp_solver.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/quadratic_program_examples.h"

using ::testing::HasSubstr;

namespace drake {
namespace solvers {
namespace test {

GTEST_TEST(QPtest, TestUnconstrainedQP) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  prog.AddQuadraticCost(x(0) * x(0));

  OsqpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 0);
  }

  // Add additional quadratic costs
  prog.AddQuadraticCost((x(1) + x(2) - 2) * (x(1) + x(2) - 2));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(result.GetSolution(x(1)) + result.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 0);
  }

  // Add linear costs.
  prog.AddLinearCost(4 * x(0) + 5);
  // Now the cost is (x₀ + 2)² + (x₁ + x₂-2)² + 1
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1E-10;
    EXPECT_NEAR(result.GetSolution(x(0)), -2, tol);
    EXPECT_NEAR(result.GetSolution(x(1)) + result.GetSolution(x(2)), 2, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 1, tol);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 0);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  OsqpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    OsqpTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  OsqpSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(QPtest, TestUnbounded) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  prog.AddQuadraticCost(x(0) * x(0) + x(1));

  OsqpSolver solver;
  // The program is unbounded.
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
  }

  // Add a constraint
  prog.AddLinearConstraint(x(0) + 2 * x(2) == 2);
  prog.AddLinearConstraint(x(0) >= 0);
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kDualInfeasible);
  }
}

GTEST_TEST(QPtest, TestInfeasible) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 2);
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) >= 2);

  OsqpSolver solver;
  // The program is infeasible.
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 0);
  }
}

GTEST_TEST(OsqpSolverTest, DualSolution1) {
  // Test GetDualSolution().
  OsqpSolver solver;
  TestQPDualSolution1(solver);
}

GTEST_TEST(OsqpSolverTest, DualSolution2) {
  // Test GetDualSolution().
  // This QP has non-zero dual solution for linear inequality constraint.
  OsqpSolver solver;
  TestQPDualSolution2(solver);
}

GTEST_TEST(OsqpSolverTest, DualSolution3) {
  // Test GetDualSolution().
  // This QP has non-zero dual solution for the bounding box constraint.
  OsqpSolver solver;
  TestQPDualSolution3(solver);
}

GTEST_TEST(OsqpSolverTest, EqualityConstrainedQPDualSolution1) {
  OsqpSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(OsqpSolverTest, EqualityConstrainedQPDualSolution2) {
  OsqpSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

GTEST_TEST(OsqpSolverTest, SolverOptionsTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddLinearConstraint(x(0) + 2 * x(1) - 3 * x(2) <= 3);
  prog.AddLinearConstraint(4 * x(0) - 2 * x(1) - 6 * x(2) >= -3);
  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1) + 5 * x(2) * x(2) +
                        2 * x(1) * x(2));
  prog.AddLinearConstraint(8 * x(0) - x(1) == 2);

  MathematicalProgramResult result;
  OsqpSolver osqp_solver;
  if (osqp_solver.available()) {
    osqp_solver.Solve(prog, {}, {}, &result);
    const int OSQP_SOLVED = 1;
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
    // OSQP is not very accurate, use a loose tolerance.
    EXPECT_TRUE(CompareMatrices(result.get_solver_details<OsqpSolver>().y,
                                Eigen::Vector3d(0, 0, -0.0619621), 1E-5));

    // Now only allow half the iterations in the OSQP solver. The solver should
    // not be able to solve the problem accurately.
    const int half_iterations =
        result.get_solver_details<OsqpSolver>().iter / 2;
    SolverOptions solver_options;
    solver_options.SetOption(osqp_solver.solver_id(), "max_iter",
                             half_iterations);
    osqp_solver.Solve(prog, {}, solver_options, &result);
    EXPECT_NE(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);

    // Now set the options in prog.
    prog.SetSolverOption(osqp_solver.solver_id(), "max_iter", half_iterations);
    osqp_solver.Solve(prog, {}, {}, &result);
    EXPECT_NE(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
  }
}

/* Tests the solver's processing of the verbosity options. With multiple ways
 to request verbosity (common options and solver-specific options), we simply
 apply a smoke test that none of the means causes runtime errors. Note, we
 don't test the case where we configure the mathematical program itself; that
 is resolved in SolverBase. We only need to test the options passed into
 Solve(). The possible configurations are:
    - No verbosity set at all (this is implicitly tested in all other tests).
    - Common option explicitly set (on & off)
    - Solver option explicitly set (on & off)
    - Both options explicitly set (with all permutations of (on, on), etc.) */
GTEST_TEST(OsqpSolverTest, SolverOptionsVerbosity) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) <= 3);
  prog.AddLinearConstraint(4 * x(0) - 2 * x(1) >= -3);
  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1) + 2 * x(0) * x(1));

  OsqpSolver osqp_solver;

  if (osqp_solver.is_available()) {
    // Setting common options.
    for (int print_to_console : {0, 1}) {
      SolverOptions options;
      options.SetOption(CommonSolverOption::kPrintToConsole, print_to_console);
      osqp_solver.Solve(prog, {}, options);
    }
    // Setting solver options.
    for (int print_to_console : {0, 1}) {
      SolverOptions options;
      options.SetOption(OsqpSolver::id(), "verbose", print_to_console);
      osqp_solver.Solve(prog, {}, options);
    }
    // Setting both.
    for (int common_print_to_console : {0, 1}) {
      for (int solver_print_to_console : {0, 1}) {
        SolverOptions options;
        options.SetOption(CommonSolverOption::kPrintToConsole,
                          common_print_to_console);
        options.SetOption(OsqpSolver::id(), "verbose", solver_print_to_console);
        osqp_solver.Solve(prog, {}, options);
      }
    }
  }
}

GTEST_TEST(OsqpSolverTest, TimeLimitTest) {
  // Intentionally create a slightly big problem to get a longer solve time.
  int n_x = 200;

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(n_x);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n_x, n_x);
  Eigen::VectorXd b = Eigen::VectorXd::Ones(n_x);
  prog.AddLinearConstraint(A, -b, b, x);
  prog.AddQuadraticErrorCost(A, -1.1 * b, x);

  MathematicalProgramResult result;
  OsqpSolver osqp_solver;
  if (osqp_solver.available()) {
    osqp_solver.Solve(prog, {}, {}, &result);
    // Status codes listed in
    // https://osqp.org/docs/interfaces/status_values.html
    const int OSQP_SOLVED = 1;
    const int OSQP_TIME_LIMIT_REACHED = -6;
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
    // OSQP is not very accurate, use a loose tolerance.
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), -b, 1E-5));

    // Now only allow one tenth of the solve time in the OSQP solver. The solver
    // should not be able to solve the problem in time.
    const double one_tenth_solve_time =
        result.get_solver_details<OsqpSolver>().solve_time / 10;
    SolverOptions solver_options;
    solver_options.SetOption(osqp_solver.solver_id(), "time_limit",
                             one_tenth_solve_time);
    osqp_solver.Solve(prog, {}, solver_options, &result);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val,
              OSQP_TIME_LIMIT_REACHED);

    // Now set the options in prog.
    prog.SetSolverOption(osqp_solver.solver_id(), "time_limit",
                         one_tenth_solve_time);
    osqp_solver.Solve(prog, {}, {}, &result);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val,
              OSQP_TIME_LIMIT_REACHED);
  }
}

GTEST_TEST(OsqpSolverTest, ProgramAttributesGood) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(x(0) * x(0));

  EXPECT_TRUE(OsqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_EQ(OsqpSolver::UnsatisfiedProgramAttributes(prog), "");
}

GTEST_TEST(OsqpSolverTest, ProgramAttributesBad) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>("x");
  prog.AddCost(x(0) * x(0) * x(0));
  EXPECT_FALSE(OsqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_THAT(OsqpSolver::UnsatisfiedProgramAttributes(prog),
              HasSubstr("GenericCost was declared"));
}

GTEST_TEST(OsqpSolverTest, ProgramAttributesMisfit) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>("x");
  prog.AddLinearCost(4 * x(0) + 5);
  EXPECT_FALSE(OsqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_THAT(OsqpSolver::UnsatisfiedProgramAttributes(prog),
              HasSubstr("QuadraticCost is required"));
}

GTEST_TEST(OsqpSolverTest, TestNonconvexQP) {
  OsqpSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, true);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
