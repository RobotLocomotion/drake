#include "drake/solvers/gurobi_solver.h"

#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mixed_integer_optimization_util.h"
#include "drake/solvers/test/linear_program_examples.h"
#include "drake/solvers/test/quadratic_program_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

TEST_P(LinearProgramTest, TestLP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    GurobiTest, LinearProgramTest,
    ::testing::Combine(::testing::ValuesIn(linear_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(linear_problems())));

TEST_F(InfeasibleLinearProgramTest0, TestGurobiInfeasible) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, Gurobi may not be able to differentiate between
    // infeasible and unbounded.
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 1);
    auto result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasible_Or_Unbounded);
    EXPECT_TRUE(std::isnan(result.get_optimal_cost()));
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
    result = solver.Solve(*prog_, {}, {});
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_TRUE(std::isinf(result.get_optimal_cost()));
    EXPECT_GE(result.get_optimal_cost(), 0);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestGurobiUnbounded) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, Gurobi may not be able to differentiate between
    // infeasible and unbounded.
    SolverOptions solver_options;
    solver_options.SetOption(GurobiSolver::id(), "DualReductions", 1);
    auto result = solver.Solve(*prog_, {}, solver_options);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasible_Or_Unbounded);
    // This code is defined in
    // https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html
    const int GRB_INF_OR_UNBD = 4;
    EXPECT_EQ(result.get_solver_details<GurobiSolver>().optimization_status,
              GRB_INF_OR_UNBD);

    solver_options.SetOption(GurobiSolver::id(), "DualReductions", 0);
    result = solver.Solve(*prog_, {}, solver_options);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    // This code is defined in
    // https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html
    const int GRB_UNBOUNDED = 5;
    EXPECT_EQ(result.get_solver_details<GurobiSolver>().optimization_status,
              GRB_UNBOUNDED);
    EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    GurobiTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(QPtest, TestUnitBallExample) {
  GurobiSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(GurobiTest, TestInitialGuess) {
  GurobiSolver solver;
  if (solver.available()) {
    // Formulate a simple problem with multiple optimal
    // solutions, and solve it twice with two different
    // initial conditions. The resulting solutions should
    // match the initial conditions supplied. Doing two
    // solves from different initial positions ensures the
    // test doesn't pass by chance.
    MathematicalProgram prog;
    auto x = prog.NewBinaryVariables<1>("x");
    // Presolve and Heuristics would each independently solve
    // this problem inside of the Gurobi solver, but without
    // consulting the initial guess.
    prog.SetSolverOption(GurobiSolver::id(), "Presolve", 0);
    prog.SetSolverOption(GurobiSolver::id(), "Heuristics", 0.0);

    double x_expected0_to_test[] = {0.0, 1.0};
    for (int i = 0; i < 2; i++) {
      Eigen::VectorXd x_expected(1);
      x_expected[0] = x_expected0_to_test[i];
      prog.SetInitialGuess(x, x_expected);
      auto result = solver.Solve(prog, x_expected, {});
      EXPECT_TRUE(result.is_success());
      const auto& x_value = result.GetSolution(x);
      EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                  MatrixCompareType::absolute));
      EXPECT_NEAR(result.get_optimal_cost(), 0, 1E-6);
    }
  }
}

namespace TestCallbacks {

struct TestCallbackInfo {
  Eigen::VectorXd x_vals;
  VectorXDecisionVariable x_vars;
  bool mip_sol_callback_called = false;
  bool mip_node_callback_called = false;
};

static void MipSolCallbackFunctionTest(
    const MathematicalProgram& prog,
    const drake::solvers::GurobiSolver::SolveStatusInfo& solve_info,
    TestCallbackInfo* cb_info) {
  cb_info->mip_sol_callback_called = true;
}
static void MipNodeCallbackFunctionTest(
    const MathematicalProgram& prog,
    const GurobiSolver::SolveStatusInfo& solve_info, Eigen::VectorXd* vals,
    VectorXDecisionVariable* vars, TestCallbackInfo* cb_info) {
  cb_info->mip_node_callback_called = true;
  *vals = cb_info->x_vals;
  *vars = cb_info->x_vars;
}

GTEST_TEST(GurobiTest, TestCallbacks) {
  GurobiSolver solver;

  if (solver.available()) {
    // Formulate a problem with multiple feasible
    // solutions and multiple clear optimal solutions.
    MathematicalProgram prog;
    auto x = prog.NewBinaryVariables<4>("x");

    // Constraint such that x_0 and x_1 can't both be
    // 1, but leave a feasible vertex at (2/3, 2/3)
    // that is optimal in the continuous relaxation.
    prog.AddLinearConstraint(x[0] <= 1. - 0.5 * x[1]);
    prog.AddLinearConstraint(x[1] <= 1. - 0.5 * x[0]);
    prog.AddLinearCost(-x[0] - x[1]);

    // Each of these options would short-circuit the solver
    // from entering a full solve and generating both
    // feasible solution callbacks (mipSol) and intermediate
    // node callbacks (mipNode).
    // Prevents the problem from being simplified, making the
    // solution potentially trivial:
    prog.SetSolverOption(GurobiSolver::id(), "Presolve", 0);
    // Prevents the optimal solution from being generated without
    // doing a full solve:
    prog.SetSolverOption(GurobiSolver::id(), "Heuristics", 0.0);
    // Similarly, prevents trivialization of the problem via
    // clever new cuts:
    prog.SetSolverOption(GurobiSolver::id(), "Cuts", 0);
    // Prevents the root node from finding the optimal feasible
    // solution via simplex, by switching to a barrier method:
    prog.SetSolverOption(GurobiSolver::id(), "NodeMethod", 2);

    // Force us to start at a known-suboptimal sol.
    Eigen::VectorXd x_init(4);
    x_init << 0.0, 0.0, 0.0, 0.0;
    prog.SetInitialGuess(x, x_init);

    // Enumerate a few different optimal solutions and try
    // injecting each of them to make sure the solver
    // is receiving these injections and listening to them.
    std::vector<Eigen::VectorXd> optimal_sols(3, Eigen::VectorXd(4));
    optimal_sols[0] << 1.0, 0.0, 0.0, 1.0;
    optimal_sols[1] << 1.0, 0.0, 1.0, 0.0;
    optimal_sols[2] << 0.0, 1.0, 1.0, 1.0;

    for (const auto& x_expected : optimal_sols) {
      TestCallbackInfo cb_info;
      cb_info.x_vals = x_expected;
      cb_info.x_vars = x;

      GurobiSolver::MipNodeCallbackFunction mip_node_callback_function_wrapper =
          std::bind(MipNodeCallbackFunctionTest, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3,
                    std::placeholders::_4, &cb_info);
      GurobiSolver::MipSolCallbackFunction mip_sol_callback_function_wrapper =
          std::bind(MipSolCallbackFunctionTest, std::placeholders::_1,
                    std::placeholders::_2, &cb_info);
      solver.AddMipNodeCallback(mip_node_callback_function_wrapper);
      solver.AddMipSolCallback(mip_sol_callback_function_wrapper);

      auto result = solver.Solve(prog, {}, {});
      EXPECT_TRUE(result.is_success());
      const auto& x_value = result.GetSolution(x);
      EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                  MatrixCompareType::absolute));
      ExpectSolutionCostAccurate(prog, result, 1E-6);
      EXPECT_TRUE(cb_info.mip_sol_callback_called);
      EXPECT_TRUE(cb_info.mip_node_callback_called);
    }
  }
}
}  // namespace TestCallbacks

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver, 1.1E-8);
  }
}

INSTANTIATE_TEST_SUITE_P(GurobiTest, TestEllipsoidsSeparation,
                        ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver);
  }
}

INSTANTIATE_TEST_SUITE_P(GurobiTest, TestQPasSOCP,
                        ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver, 2E-2);
  }
}

INSTANTIATE_TEST_SUITE_P(
    GurobiTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem1) {
  MaximizeGeometricMeanTrivialProblem1 prob;
  GurobiSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    prob.CheckSolution(result, 4E-6);
  }
}

GTEST_TEST(TestSOCP, MaximizeGeometricMeanTrivialProblem2) {
  MaximizeGeometricMeanTrivialProblem2 prob;
  GurobiSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prob.prog(), {}, {});
    // Gurobi 9.0.0 returns a solution that is accurate up to 1.4E-6 for this
    // specific problem. Might need to change the tolerance when we upgrade
    // Gurobi.
    prob.CheckSolution(result, 1.4E-6);
  }
}

GTEST_TEST(TestSOCP, SmallestEllipsoidCoveringProblem) {
  GurobiSolver solver;
  SolveAndCheckSmallestEllipsoidCoveringProblems(solver, 1E-6);
}

GTEST_TEST(GurobiTest, MultipleThreadsSharingEnvironment) {
  // Running multiple threads of GurobiSolver, they share the same GRBenv
  // which is created when acquiring the Gurobi license in the main function.

  auto solve_program = [](int i, int N) {
    // We want to solve a complicated program in each thread, so that multiple
    // programs will run concurrently. To this end, in each thread, we solve
    // the following mixed-integer program
    // min (x - i)² + (y - 1)²
    // s.t Point (x, y) are on the line segments A₁A₂, A₂A₃, ..., Aₙ₋₁Aₙ,
    // where A₂ⱼ= (2j, 1), A₂ⱼ₊₁ = (2j+1, 0)
    // When i is an even number, the optimal solution is (i, 1), with optimal
    // cost equals to 0. When i is an odd number, the optimal solution is either
    // (i - 0.5, 0.5) or (i + 0.5, 0.5), with the optimal cost being 0.5
    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<1>()(0);
    auto y = prog.NewContinuousVariables<1>()(0);
    // To constrain the point (x, y) on the line segment, we introduce a SOS2
    // constraint with auxiliary variable lambda.
    auto lambda = prog.NewContinuousVariables(N + 1);
    // TODO(hongkai.dai): there is a bug in AddSos2Constraint, that it didn't
    // add lambda(N) >= 0. After I resolve that bug, the next line could be
    // removed.
    prog.AddBoundingBoxConstraint(0, 1, lambda);
    auto z = prog.NewBinaryVariables(N);
    AddSos2Constraint(&prog, lambda.cast<symbolic::Expression>(),
                      z.cast<symbolic::Expression>());
    Vector2<symbolic::Expression> line_segment(0, 0);
    for (int j = 0; j <= N; ++j) {
      line_segment(0) += j * lambda(j);
      line_segment(1) += j % 2 == 0 ? symbolic::Expression(lambda(j))
                                    : symbolic::Expression(0);
    }
    prog.AddLinearConstraint(line_segment(0) == x);
    prog.AddLinearConstraint(line_segment(1) == y);
    prog.AddQuadraticCost((x - i) * (x - i) + (y - 1) * (y - 1));
    GurobiSolver gurobi_solver;
    auto result = gurobi_solver.Solve(prog, {}, {});
    EXPECT_TRUE(result.is_success());

    const double tol = 1E-6;
    if (i % 2 == 0) {
      EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
      EXPECT_NEAR(result.GetSolution(x), i, tol);
      EXPECT_NEAR(result.GetSolution(y), 1, tol);
    } else {
      EXPECT_NEAR(result.get_optimal_cost(), 0.5, tol);
      EXPECT_NEAR(result.GetSolution(y), 0.5, tol);
      const double x_val = result.GetSolution(x);
      EXPECT_TRUE(std::abs(x_val - (i - 0.5)) < tol ||
                  std::abs(x_val - (i + 0.5)) < tol);
    }
  };

  std::vector<std::thread> test_threads;
  const int num_threads = 20;
  for (int i = 0; i < num_threads; ++i) {
    test_threads.emplace_back(solve_program, i, num_threads);
  }
  for (int i = 0; i < num_threads; ++i) {
    test_threads[i].join();
  }
}

GTEST_TEST(GurobiTest, GurobiErrorCode) {
  // This test verifies that we can return the error code reported by Gurobi.

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x(0) + x(1) <= 1);

  GurobiSolver solver;
  if (solver.available()) {
    SolverOptions solver_options1;
    // Report error when we set an unknown attribute to Gurobi.
    solver_options1.SetOption(solver.solver_id(), "Foo", 1);
    DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, {}, solver_options1),
                                ".* 'Foo' is an unknown parameter in Gurobi.*");
    // Report error when we pass an incorect value to a valid Gurobi parameter
    SolverOptions solver_options2;
    solver_options2.SetOption(solver.solver_id(), "FeasibilityTol", 1E10);
    DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, {}, solver_options2),
                                ".* is outside the parameter Feasibility.*");
  }
}

GTEST_TEST(GurobiTest, LogFile) {
  // Test setting gurobi log file.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  prog.AddQuadraticCost(x[0] * x[0] + 2 * x[1] * x[1]);
  prog.AddBoundingBoxConstraint(0, 1, x);
  prog.AddLinearEqualityConstraint(x[0] + x[1] + 2 * x[2] == 1);
  prog.NewBinaryVariables<2>();

  GurobiSolver solver;
  if (solver.available()) {
    {
      SolverOptions solver_options;
      const std::string log_file = temp_directory() + "/gurobi.log";
      EXPECT_FALSE(filesystem::exists({log_file}));
      solver_options.SetOption(solver.id(), "LogFile", log_file);
      auto result = solver.Solve(prog, {}, solver_options);
      EXPECT_TRUE(filesystem::exists({log_file}));
    }

    // Set log file through CommonSolverOptions.
    {
      SolverOptions solver_options;
      const std::string log_file_common =
          temp_directory() + "/gurobi_common.log";
      EXPECT_FALSE(filesystem::exists({log_file_common}));
      solver_options.SetOption(CommonSolverOption::kPrintFileName,
                               log_file_common);
      solver.Solve(prog, {}, solver_options);
      EXPECT_TRUE(filesystem::exists({log_file_common}));
    }

    // Also set to log to console. We can't test the console output but this
    // test verifies no error thrown.
    {
      SolverOptions solver_options;
      solver_options.SetOption(CommonSolverOption::kPrintToConsole, 1);
      solver_options.SetOption(CommonSolverOption::kPrintFileName, "");
      auto result = solver.Solve(prog, {}, solver_options);
      EXPECT_TRUE(result.is_success());
    }

    // Set the option through both CommonSolverOption and solver-specific
    // option. The common solver option should win.
    {
      SolverOptions solver_options;
      const std::string log_file_common =
          temp_directory() + "/gurobi_common2.log";
      solver_options.SetOption(CommonSolverOption::kPrintFileName,
                               log_file_common);
      const std::string log_file = temp_directory() + "/gurobi2.log";
      solver_options.SetOption(solver.id(), "LogFile", log_file);
      EXPECT_FALSE(filesystem::exists({log_file}));
      EXPECT_FALSE(filesystem::exists({log_file_common}));
      auto result = solver.Solve(prog, {}, solver_options);
      EXPECT_TRUE(filesystem::exists({log_file}));
      EXPECT_FALSE(filesystem::exists({log_file_common}));
    }
  }
}

GTEST_TEST(GurobiTest, WriteModel) {
  // Test writing Gurobi model to a file.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] == 1);
  prog.AddQuadraticCost(x[0] * x[0] + x[1] * x[1]);

  GurobiSolver solver;
  if (solver.available()) {
    const std::string model_file = temp_directory() + "/gurobi_model.mps";
    SolverOptions options;
    options.SetOption(solver.id(), "GRBwrite", "");
    // Setting GRBwrite to "" and make sure calling Solve doesn't cause error.
    solver.Solve(prog, {}, options);
    options.SetOption(solver.id(), "GRBwrite", model_file);
    EXPECT_FALSE(filesystem::exists({model_file}));
    const auto result = solver.Solve(prog, {}, options);
    EXPECT_TRUE(filesystem::exists({model_file}));
    options.SetOption(solver.id(), "GRBwrite", "foo.wrong_extension");
    DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, {}, options),
                                ".* setting GRBwrite to foo.wrong_extension.*");
  }
}

GTEST_TEST(GurobiTest, ComputeIIS) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(x[0] + x[1] == 2);
  prog.AddLinearConstraint(x[0] - x[1] == 0);
  auto bb_con = prog.AddBoundingBoxConstraint(2, 10, x[0]);

  GurobiSolver solver;
  if (solver.available()) {
    SolverOptions options;
    options.SetOption(solver.id(), "GRBcomputeIIS", 1);
    const std::string ilp_file = temp_directory() + "/gurobi_model.ilp";
    options.SetOption(solver.id(), "GRBwrite", ilp_file);
    EXPECT_FALSE(filesystem::exists({ilp_file}));
    auto result = solver.Solve(prog, {}, options);
    EXPECT_TRUE(filesystem::exists({ilp_file}));
    // Set GRBcomputeIIS to a wrong value.
    options.SetOption(solver.id(), "GRBcomputeIIS", 100);
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Solve(prog, {}, options),
        ".*option GRBcomputeIIS should be either 0 or 1.*");
    // Reset GRBcomputeIIS to the right value.
    options.SetOption(solver.id(), "GRBcomputeIIS", 1);

    // Now remove bb_con. The problem should be feasible.
    prog.RemoveConstraint(bb_con);
    options.SetOption(solver.id(), "GRBwrite", "");
    result = solver.Solve(prog, {}, options);
    EXPECT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x), Eigen::Vector2d(1, 1)));
  }
}

GTEST_TEST(GurobiTest, SolutionPool) {
  // For mixed-integer program, Gurobi can find a pool of suboptimal solutions.
  MathematicalProgram prog;
  auto b = prog.NewBinaryVariables<2>();
  prog.AddLinearEqualityConstraint(b(0) + b(1) == 1);
  prog.AddLinearCost(b(0));

  GurobiSolver solver;
  if (solver.is_available()) {
    SolverOptions solver_options;
    // Find at most 3 suboptimal solutions. Note that the problem only has 2
    // solutions. This is to make sure that the user can set the size of the
    // pool as large as he wants, and the solver will try to find all possible
    // solutions.
    solver_options.SetOption(solver.id(), "PoolSolutions", 3);
    MathematicalProgramResult result;
    solver.Solve(prog, {}, solver_options, &result);
    // The problem has only two set of solutions, either b = [0, 1] and b = [1,
    // 0].
    EXPECT_EQ(result.num_suboptimal_solution(), 2);
    const double tol = 1E-8;
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(b), Eigen::Vector2d(0, 1), tol));
    EXPECT_TRUE(CompareMatrices(result.GetSuboptimalSolution(b, 0),
                                Eigen::Vector2d(0, 1), tol));
    EXPECT_TRUE(CompareMatrices(result.GetSuboptimalSolution(b, 1),
                                Eigen::Vector2d(1, 0), tol));
    EXPECT_NEAR(result.get_optimal_cost(), 0, tol);
    EXPECT_NEAR(result.get_suboptimal_objective(0), 0, tol);
    EXPECT_NEAR(result.get_suboptimal_objective(1), 1, tol);
  }
}

GTEST_TEST(GurobiTest, QPDualSolution1) {
  GurobiSolver solver;
  TestQPDualSolution1(solver, {} /* solver_options */, 1e-6);
}

GTEST_TEST(GurobiTest, QPDualSolution2) {
  GurobiSolver solver;
  TestQPDualSolution2(solver);
}

GTEST_TEST(GurobiTest, QPDualSolution3) {
  GurobiSolver solver;
  TestQPDualSolution3(solver);
}

GTEST_TEST(GurobiTest, EqualityConstrainedQPDualSolution1) {
  GurobiSolver solver;
  TestEqualityConstrainedQPDualSolution1(solver);
}

GTEST_TEST(GurobiTest, EqualityConstrainedQPDualSolution2) {
  GurobiSolver solver;
  TestEqualityConstrainedQPDualSolution2(solver);
}

GTEST_TEST(GurobiTest, LPDualSolution1) {
  GurobiSolver solver;
  TestLPDualSolution1(solver);
}

GTEST_TEST(GurobiTest, LPDualSolution2) {
  GurobiSolver solver;
  TestLPDualSolution2(solver);
}

GTEST_TEST(GurobiTest, LPDualSolution3) {
  GurobiSolver solver;
  TestLPDualSolution3(solver);
}

GTEST_TEST(GurobiTest, SOCPDualSolution1) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto constraint1 = prog.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(2., 2 * x(0), 3 * x(1) + 1));
  GurobiSolver solver;
  prog.AddLinearCost(x(1));
  if (solver.is_available()) {
    // By default the dual solution for second order cone is not computed.
    MathematicalProgramResult result = solver.Solve(prog);
    DRAKE_EXPECT_THROWS_MESSAGE(
        result.GetDualSolution(constraint1), std::invalid_argument,
        "You used Gurobi to solve this optimization problem.*");
    SolverOptions options;
    options.SetOption(solver.id(), "QCPDual", 1);
    result = solver.Solve(prog, std::nullopt, options);
    // The shadow price can be computed analytically, since the optimal cost
    // is (-sqrt(4 + eps) - 1)/3, when the Lorentz cone constraint is perturbed
    // by eps as 2*x(0)² + (3*x(1)+1)² <= 4 + eps. The gradient of the optimal
    // cost (-sqrt(4 + eps) - 1)/3 w.r.t eps is -1/12.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1),
                                Vector1d(-1. / 12), 1e-7));

    // Now add a bounding box constraint to the program. By setting QCPDual to
    // 0, the program should throw an error.
    auto bb_con = prog.AddBoundingBoxConstraint(0, kInf, x(1));
    options.SetOption(solver.id(), "QCPDual", 0);
    result = solver.Solve(prog, std::nullopt, options);
    DRAKE_EXPECT_THROWS_MESSAGE(
        result.GetDualSolution(bb_con), std::invalid_argument,
        "You used Gurobi to solve this optimization problem.*");
    // Now set QCPDual = 1, we should be able to retrieve the dual solution to
    // the bounding box constraint.
    options.SetOption(solver.id(), "QCPDual", 1);
    result = solver.Solve(prog, std::nullopt, options);
    // The cost is x(1), hence the shadow price for the constraint x(1) >= 0
    // should be 1.
    EXPECT_TRUE(
        CompareMatrices(result.GetDualSolution(bb_con), Vector1d(1.), 1E-8));
  }
}

GTEST_TEST(GurobiTest, SOCPDualSolution2) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>()(0);
  auto constraint1 = prog.AddRotatedLorentzConeConstraint(
      Vector3<symbolic::Expression>(2., x + 1.5, x));
  auto constraint2 =
      prog.AddLorentzConeConstraint(Vector2<symbolic::Expression>(1, x + 1));
  prog.AddLinearCost(x);
  GurobiSolver solver;
  if (solver.is_available()) {
    SolverOptions options;
    options.SetOption(GurobiSolver::id(), "QCPDual", 1);
    const auto result = solver.Solve(prog, {}, options);
    // By pertubing the constraint1 as x^2 <= 2x + 3 + eps, the optimal cost
    // becomes -1 - sqrt(4+eps). The gradient of the cost w.r.t eps is -1/4.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint1),
                                Vector1d(-1.0 / 4), 1e-8));
    // constraint 2 is not active at the optimal solution, hence the shadow
    // price is 0.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(constraint2),
                                Vector1d(0), 1e-8));
  }
}

GTEST_TEST(GurobiTest, TestNonconvexQP) {
  GurobiSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, true);
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the Gurobi license for the entire duration of this
  // test, so that we do not have to release and re-acquire the license for
  // every test.
  auto gurobi_license = drake::solvers::GurobiSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
