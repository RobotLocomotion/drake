#include "drake/solvers/gurobi_solver.h"

#include <thread>

#include <gtest/gtest.h>

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

TEST_P(LinearProgramTest, TestLP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
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
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasible_Or_Unbounded);
    EXPECT_TRUE(std::isnan(prog_->GetOptimalCost()));
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
    result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
    EXPECT_TRUE(std::isinf(prog_->GetOptimalCost()));
    EXPECT_GE(prog_->GetOptimalCost(), 0);
  }
}

TEST_F(UnboundedLinearProgramTest0, TestGurobiUnbounded) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, Gurobi may not be able to differentiate between
    // infeasible and unbounded.
    SolverOptions solver_options;
    solver_options.SetOption(GurobiSolver::id(), "DualReductions", 1);
    MathematicalProgramResult result;
    solver.Solve(*prog_, {}, solver_options, &result);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasible_Or_Unbounded);
    // This code is defined in
    // https://www.gurobi.com/documentation/8.0/refman/optimization_status_codes.html
    const int GRB_INF_OR_UNBD = 4;
    EXPECT_EQ(result.get_solver_details()
                  .GetValue<GurobiSolverDetails>()
                  .optimization_status,
              GRB_INF_OR_UNBD);

    solver_options.SetOption(GurobiSolver::id(), "DualReductions", 0);
    solver.Solve(*prog_, {}, solver_options, &result);
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kUnbounded);
    // This code is defined in
    // https://www.gurobi.com/documentation/8.0/refman/optimization_status_codes.html
    const int GRB_UNBOUNDED = 5;
    EXPECT_EQ(result.get_solver_details()
                  .GetValue<GurobiSolverDetails>()
                  .optimization_status,
              GRB_UNBOUNDED);
    EXPECT_EQ(result.get_optimal_cost(), MathematicalProgram::kUnboundedCost);
  }
}

TEST_P(QuadraticProgramTest, TestQP) {
  GurobiSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_CASE_P(
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
    MathematicalProgramResult result;
    for (int i = 0; i < 2; i++) {
      Eigen::VectorXd x_expected(1);
      x_expected[0] = x_expected0_to_test[i];
      prog.SetInitialGuess(x, x_expected);
      solver.Solve(prog, x_expected, {}, &result);
      EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
      const auto& x_value = prog.GetSolution(x, result);
      EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                  MatrixCompareType::absolute));
      EXPECT_NEAR(result.get_optimal_cost(), 0, 1E-6);
    }

    // Set wrong initial guess with incorrect size.
    Eigen::VectorXd initial_guess_wrong_size(2);
    DRAKE_EXPECT_THROWS_MESSAGE(
        solver.Solve(prog, initial_guess_wrong_size, {}, &result),
        std::invalid_argument,
        "The initial guess has 2 rows, but 1 rows were expected.");
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

      MathematicalProgramResult result;
      solver.Solve(prog, {}, {}, &result);
      EXPECT_EQ(result.get_solution_result(), SolutionResult::kSolutionFound);
      const auto& x_value = prog.GetSolution(x, result);
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

INSTANTIATE_TEST_CASE_P(GurobiTest, TestEllipsoidsSeparation,
                        ::testing::ValuesIn(GetEllipsoidsSeparationProblems()));

TEST_P(TestQPasSOCP, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver);
  }
}

INSTANTIATE_TEST_CASE_P(GurobiTest, TestQPasSOCP,
                        ::testing::ValuesIn(GetQPasSOCPProblems()));

TEST_P(TestFindSpringEquilibrium, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver, 2E-2);
  }
}

INSTANTIATE_TEST_CASE_P(
    GurobiTest, TestFindSpringEquilibrium,
    ::testing::ValuesIn(GetFindSpringEquilibriumProblems()));

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
    const SolutionResult result = gurobi_solver.Solve(prog);
    EXPECT_EQ(result, SolutionResult::kSolutionFound);

    const double tol = 1E-6;
    if (i % 2 == 0) {
      EXPECT_NEAR(prog.GetOptimalCost(), 0, tol);
      EXPECT_NEAR(prog.GetSolution(x), i, tol);
      EXPECT_NEAR(prog.GetSolution(y), 1, tol);
    } else {
      EXPECT_NEAR(prog.GetOptimalCost(), 0.5, tol);
      EXPECT_NEAR(prog.GetSolution(y), 0.5, tol);
      const double x_val = prog.GetSolution(x);
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
    MathematicalProgramResult result;
    SolverOptions solver_options;
    // Report error when we set an unknown attribute to Gurobi.
    solver_options.SetOption(solver.solver_id(), "Foo", 1);
    solver.Solve(prog, {}, solver_options, &result);
    // The error code is listed in
    // https://www.gurobi.com/documentation/8.0/refman/error_codes.html
    const int UNKNOWN_PARAMETER{10007};
    EXPECT_EQ(
        result.get_solver_details().GetValue<GurobiSolverDetails>().error_code,
        UNKNOWN_PARAMETER);

    // Report error if the Q matrix in the QP cost is not positive semidefinite.
    prog.AddQuadraticCost(x(0) * x(0) - x(1) * x(1));
    solver.Solve(prog, {}, {}, &result);
    // The error code is listed in
    // https://www.gurobi.com/documentation/8.0/refman/error_codes.html
    const int Q_NOT_PSD{10020};
    EXPECT_EQ(
        result.get_solver_details().GetValue<GurobiSolverDetails>().error_code,
        Q_NOT_PSD);
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
