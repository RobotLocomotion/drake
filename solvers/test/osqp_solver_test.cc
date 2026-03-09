#include "drake/solvers/osqp_solver.h"

#include <limits>

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
    const double tol = 1e-4;
    EXPECT_NEAR(result.GetSolution(x(0)), 0, tol);
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 0);
  }

  // Add additional quadratic costs
  prog.AddQuadraticCost((x(1) + x(2) - 2) * (x(1) + x(2) - 2));
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());
    const double tol = 1e-4;
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
    const double tol = 1e-4;
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
    EXPECT_TRUE(result.GetSolution(x).array().isFinite().all());
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.size(), 0);
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
  auto constraint0 = prog.AddLinearConstraint(x(0) + 2 * x(1) == 2);
  auto constraint1 = prog.AddLinearConstraint(x(0) >= 1);
  auto constraint2 = prog.AddLinearConstraint(x(1) >= 2);

  OsqpSolver solver;
  // The program is infeasible.
  if (solver.available()) {
    auto result = solver.Solve(prog, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);

    EXPECT_EQ(result.get_solver_details<OsqpSolver>().y.rows(), 3);
    // Primal solution is not NAN or inf.
    EXPECT_TRUE(result.GetSolution(x).array().isFinite().all());
    // Dual solution is not NAN or inf.
    EXPECT_TRUE(
        result.get_solver_details<OsqpSolver>().y.array().isFinite().all());
    EXPECT_TRUE(result.GetDualSolution(constraint0).array().isFinite().all());
    EXPECT_TRUE(result.GetDualSolution(constraint1).array().isFinite().all());
    EXPECT_TRUE(result.GetDualSolution(constraint2).array().isFinite().all());
    // In OSQP's default upstream settings, time-based adaptive rho is enabled
    // by default (i.e., adaptive_rho_interval=0). However, in our OsqpSolver
    // wrapper, we've changed the default to be non-zero so that solver results
    // are deterministic. The following check proves that our custom default is
    // effective: with time-based adaptive rho there would be one rho_update,
    // but with our custom default value there will be no updates (since the
    // iteration count never reaches the scheduled iteration step of an update).
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().rho_updates, 0);
  }
}

GTEST_TEST(QPtest, TestQuadraticCostVariableOrder) {
  OsqpSolver solver;
  if (solver.available()) {
    TestQuadraticCostVariableOrder(solver);
  }
}

GTEST_TEST(OsqpSolverTest, DuplicatedVariable) {
  OsqpSolver solver;
  if (solver.available()) {
    TestDuplicatedVariableQuadraticProgram(solver, 1e-4);
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

void AddTestProgram(MathematicalProgram* prog,
                    const MatrixDecisionVariable<3, 1>& x) {
  prog->AddLinearConstraint(x(0) + 2 * x(1) - 3 * x(2) <= 3);
  prog->AddLinearConstraint(4 * x(0) - 2 * x(1) - 6 * x(2) >= -3);
  prog->AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1) + 5 * x(2) * x(2) +
                         2 * x(1) * x(2));
  prog->AddLinearConstraint(8 * x(0) - x(1) == 2);
}

// This is a regression test for #18711. To reproduce that issue, we need to
// solve a sufficiently complicated QP so that OSQP will iterate internally
// enough to engage its wall-clock timing heuristics. We'll set up some very
// specific costs and constraints (captured from a real world example) and
// then repeatedly solve the same program over and over again, checking that
// the solution is bit-exact each time. Note that this is a probabilistic test:
// even if we revert the fix for #18711, this only fails ~40% of the time.
GTEST_TEST(OsqpSolverTest, ComplicatedExampleForAdaptiveRhoTiming) {
  MathematicalProgram prog;

  const int num_v = 7;
  const int num_u = 7;

  Eigen::MatrixXd M(num_v, num_v);
  M << 1.5265000092743219, 1.3567292738062303e-01, 1.0509980442610054,
      1.0593165893494538e-01, 5.2550137200876458e-02, 1.4695302469350939e-02,
      -5.6634685479957378e-03,
      //
      1.3567292738062303e-01, 2.6582813453131502, 1.8437561004161429e-01,
      -1.1606165163872926, -6.7115703782523864e-02, -6.8744248816288878e-02,
      -2.3803654356554906e-03,
      //
      1.0509980442610054, 1.8437561004161429e-01, 7.5578462775698307e-01,
      2.9778874473806963e-02, 4.2191302344065193e-02, 6.9686779847817938e-03,
      -4.7168273440004261e-03,
      //
      1.0593165893494538e-01, -1.1606165163872926, 2.9778874473806963e-02,
      7.0143698885704575e-01, 4.3907540905320175e-02, 6.0704344118780576e-02,
      6.2450624459637065e-04,
      //
      5.2550137200876458e-02, -6.7115703782523864e-02, 4.2191302344065193e-02,
      4.3907540905320175e-02, 2.3108956248772367e-02, -4.0027553954531062e-04,
      -9.6135531130737148e-04,
      //
      1.4695302469350939e-02, -6.8744248816288878e-02, 6.9686779847817938e-03,
      6.0704344118780576e-02, -4.0027553954531062e-04, 1.8750100174003432e-02,
      -2.3318254131776553e-04,
      //
      -5.6634685479957378e-03, -2.3803654356554906e-03, -4.7168273440004261e-03,
      6.2450624459637065e-04, -9.6135531130737148e-04, -2.3318254131776553e-04,
      8.4560640585000036e-04;

  Eigen::VectorXd C(num_v);
  C << 0.12534316615346058, -4.262437098995484, -0.1292406593401465,
      1.2824026531372175, 0.057990230523857114, -0.11631318722553598,
      0.008118721429606525;

  Eigen::VectorXd tau_g(num_v);
  tau_g << 0.0, 3.2520059971872882e+01, 1.1056387788047277,
      -1.6455090196304102e+01, -1.3129246602609450, -1.2645040362872129,
      -1.6577819002396472e-02;

  auto vd_star = prog.NewContinuousVariables(num_v, "vd_star");
  auto u_star = prog.NewContinuousVariables(num_u, "u_star");

  Eigen::MatrixXd Aeq(num_v, num_v + num_v);
  Aeq.block(0, 0, num_v, num_v) = M;
  Aeq.block(0, num_v, num_v, num_v) = Eigen::MatrixXd::Identity(num_v, num_v);
  Eigen::VectorXd beq = -C + tau_g;
  prog.AddLinearEqualityConstraint(Aeq, beq, {vd_star, u_star});

  Eigen::MatrixXd task_cost_A(6, num_v);
  task_cost_A << 0.0, 2.6276770346517636e-01, 3.0114875389439555e-01,
      -4.7285620522256971e-01, 8.7440544000403608e-01, -4.8174857078266453e-01,
      -1.9618608268142981e-01,
      //
      0.0, 9.6485912651310768e-01, -8.2014217710682832e-02,
      -8.7801906757823511e-01, -4.5720465227404666e-01, -8.5793882676911759e-01,
      -9.2923842949070412e-02,
      //
      1.0, 4.8965888601467475e-12, 9.5004373379395413e-01,
      7.4091336548597356e-02, 1.6241623204075539e-01, 1.7849169188197500e-01,
      -9.7615376881600568e-01,
      //
      2.8311221691848737e-01, 2.9275947741254926e-01, 2.4408407039656915e-01,
      -8.6056328493326305e-03, 6.1561854687226442e-02, 1.0864947505835820e-01,
      -1.2143064331837650e-17,
      //
      5.4444702526853128e-01, -7.9729499810435950e-02, 4.2587333018015111e-01,
      4.1571230146216887e-02, 1.0963458655960948e-01, -4.5887476583813355e-02,
      -1.3877787807814457e-17,
      //
      0.0, -5.9970742829586077e-01, -4.0606494474492376e-02,
      4.3771792154289879e-01, -2.2809158687563048e-02, 7.2681710645204109e-02,
      3.6862873864507151e-18;
  Eigen::VectorXd task_cost_b(6);
  task_cost_b << -24.566247201232553, -0.3677519895564192, -44.03725978974725,
      34.38537004440926, 45.39628041050313, -42.53770459411413;
  Eigen::MatrixXd task_cost_proj(num_v, 6);
  task_cost_proj << 0.0, 0.0, 1.0, 2.8311221691848737e-01,
      5.4444702526853128e-01, 0.0,
      //
      2.6276770346517636e-01, 9.6485912651310768e-01, 4.8965888601467475e-12,
      2.9275947741254926e-01, -7.9729499810435950e-02, -5.9970742829586077e-01,
      //
      3.0114875389439555e-01, -8.2014217710682832e-02, 9.5004373379395413e-01,
      2.4408407039656915e-01, 4.2587333018015111e-01, -4.0606494474492376e-02,
      //
      -4.7285620522256971e-01, -8.7801906757823511e-01, 7.4091336548597356e-02,
      -8.6056328493326305e-03, 4.1571230146216887e-02, 4.3771792154289879e-01,
      //
      8.7440544000403608e-01, -4.5720465227404666e-01, 1.6241623204075539e-01,
      6.1561854687226442e-02, 1.0963458655960948e-01, -2.2809158687563048e-02,
      //
      -4.8174857078266453e-01, -8.5793882676911759e-01, 1.7849169188197500e-01,
      1.0864947505835820e-01, -4.5887476583813355e-02, 7.2681710645204109e-02,
      //
      -1.9618608268142981e-01, -9.2923842949070412e-02, -9.7615376881600568e-01,
      -1.2143064331837650e-17, -1.3877787807814457e-17, 3.6862873864507151e-18;
  prog.Add2NormSquaredCost(task_cost_proj * task_cost_A,
                           task_cost_proj * task_cost_b, vd_star);

  if (!OsqpSolver::is_available()) {
    return;
  }

  // Run one solve to get a specific answer.
  auto run_one_solve = [&prog, &u_star]() {
    const OsqpSolver dut;
    const MathematicalProgramResult result = dut.Solve(prog);
    EXPECT_TRUE(result.is_success());
    return result.GetSolution(u_star);
  };
  const Eigen::VectorXd first_answer = run_one_solve();

  // Run the same thing over and over again, to "prove" that it's deterministic.
  for (int i = 0; i < 100; ++i) {
    SCOPED_TRACE(fmt::format("i = {}", i));
    const Eigen::VectorXd new_answer = run_one_solve();
    ASSERT_EQ(new_answer, first_answer);
  }
}

GTEST_TEST(OsqpSolverTest, SolverOptionsTest) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  AddTestProgram(&prog, x);

  MathematicalProgramResult result;
  OsqpSolver osqp_solver;
  if (osqp_solver.available()) {
    osqp_solver.Solve(prog, {}, {}, &result);
    const int OSQP_SOLVED = 1;
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
    // OSQP is not very accurate, use a loose tolerance.
    EXPECT_TRUE(CompareMatrices(result.get_solver_details<OsqpSolver>().y,
                                Eigen::Vector3d(0, 0, -0.0619621), 1e-4));

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

GTEST_TEST(OsqpSolverTest, WarmStartPrimalOnly) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  AddTestProgram(&prog, x);

  MathematicalProgramResult result;
  OsqpSolver osqp_solver;
  if (osqp_solver.available()) {
    // Solve with no prior solution; should be same as above test case.
    osqp_solver.Solve(prog, {}, {}, &result);
    const int OSQP_SOLVED = 1;
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);

    // Solve with primal-only warm-start, restricting the iterations as above,
    // but showing that we now have a solution.
    const int half_iterations =
        result.get_solver_details<OsqpSolver>().iter / 2;
    SolverOptions solver_options;
    solver_options.SetOption(osqp_solver.solver_id(), "max_iter",
                             half_iterations);
    const Eigen::VectorXd x_sol = result.get_x_val();
    osqp_solver.Solve(prog, x_sol, solver_options, &result);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
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
    const int OSQP_TIME_LIMIT_REACHED = 8;
    EXPECT_TRUE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val, OSQP_SOLVED);
    // OSQP is not very accurate, use a loose tolerance.
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), -b, 1e-4));

    // Now only allow one hundredth of the solve time in the OSQP solver. The
    // solver should not be able to solve the problem in time.
    const double original_solve_time =
        result.get_solver_details<OsqpSolver>().solve_time;
    ASSERT_GT(original_solve_time, 0.0);
    const double one_hundredth_solve_time = original_solve_time / 100.0;
    SolverOptions solver_options;
    solver_options.SetOption(osqp_solver.solver_id(), "time_limit",
                             one_hundredth_solve_time);
    osqp_solver.Solve(prog, {}, solver_options, &result);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kSolverSpecificError);
    EXPECT_EQ(result.get_solver_details<OsqpSolver>().status_val,
              OSQP_TIME_LIMIT_REACHED);

    // Now set the options in prog.
    prog.SetSolverOption(osqp_solver.solver_id(), "time_limit",
                         one_hundredth_solve_time);
    osqp_solver.Solve(prog, {}, {}, &result);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kSolverSpecificError);
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

GTEST_TEST(OsqpSolverTest, VariableScaling1) {
  // Quadractic cost and linear inequality constraints.
  double s = 100;
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(2 * x(0) / s - 2 * x(1) == 2);
  prog.AddQuadraticCost((x(0) / s + 1) * (x(0) / s + 1));
  prog.AddQuadraticCost((x(1) + 1) * (x(1) + 1));

  prog.SetVariableScaling(x(0), s);

  OsqpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog);

    EXPECT_TRUE(result.is_success());
    const double tol = 1e-4;
    EXPECT_NEAR(result.get_optimal_cost(), 0.5, tol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector2d((-0.5) * s, -1.5), tol));
  }
}

GTEST_TEST(OsqpSolverTest, VariableScaling2) {
  // Quadratic and linear cost, together with bounding box constraints.
  double s = 100;
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddBoundingBoxConstraint(0.5 * s,
                                std::numeric_limits<double>::infinity(), x(0));
  prog.AddQuadraticCost((x(0) / s + 1) * (x(0) / s + 1));
  prog.AddQuadraticCost(x(1) * x(1));
  prog.AddLinearCost(2 * x(1) + 1);

  prog.SetVariableScaling(x(0), s);

  OsqpSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog);

    EXPECT_TRUE(result.is_success());
    const double tol = 1e-4;
    EXPECT_NEAR(result.get_optimal_cost(), 2.25, tol);
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector2d((0.5) * s, -1), tol));
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
