#include "drake/solvers/gurobi_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/mathematical_program.h"
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
    // With dual reductions, gurobi may not be able to differentiate between
    // infeasible and unbounded.
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 1);
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasible_Or_Unbounded);
    EXPECT_TRUE(std::isnan(prog_->GetOptimalCost()));
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
    result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasibleConstraints);
    EXPECT_TRUE(std::isnan(prog_->GetOptimalCost()));
  }
}

TEST_F(UnboundedLinearProgramTest0, TestGurobiUnbounded) {
  GurobiSolver solver;
  if (solver.available()) {
    // With dual reductions, gurobi may not be able to differentiate between
    // infeasible and unbounded.
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 1);
    SolutionResult result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kInfeasible_Or_Unbounded);
    prog_->SetSolverOption(GurobiSolver::id(), "DualReductions", 0);
    result = solver.Solve(*prog_);
    EXPECT_EQ(result, SolutionResult::kUnbounded);
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
    for (int i = 0; i < 2; i++) {
      Eigen::VectorXd x_expected(1);
      x_expected[0] = x_expected0_to_test[i];
      prog.SetInitialGuess(x, x_expected);
      SolutionResult result = solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      const auto& x_value = prog.GetSolution(x);
      EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                  MatrixCompareType::absolute));
      ExpectSolutionCostAccurate(prog, 1E-6);
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

      SolutionResult result = solver.Solve(prog);
      EXPECT_EQ(result, SolutionResult::kSolutionFound);
      const auto& x_value = prog.GetSolution(x);
      EXPECT_TRUE(CompareMatrices(x_value, x_expected, 1E-6,
                                  MatrixCompareType::absolute));
      ExpectSolutionCostAccurate(prog, 1E-6);
      EXPECT_TRUE(cb_info.mip_sol_callback_called);
      EXPECT_TRUE(cb_info.mip_node_callback_called);
    }
  }
}
}  // namespace TestCallbacks

TEST_P(TestEllipsoidsSeparation, TestSOCP) {
  GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    SolveAndCheckSolution(gurobi_solver);
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
}  // namespace test
}  // namespace solvers
}  // namespace drake
