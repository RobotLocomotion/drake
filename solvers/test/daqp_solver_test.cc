#include "drake/solvers/daqp_solver.h"

#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/quadratic_program_examples.h"

using ::testing::HasSubstr;

namespace drake {
namespace solvers {
namespace test {
namespace {

// DAQP is an active-set method, so its solutions are accurate to near machine
// precision; we can use much tighter tolerances than for OSQP.
constexpr double kTol = 1e-8;

TEST_P(QuadraticProgramTest, TestQP) {
  DaqpSolver solver;
  prob()->RunProblem(&solver);
}

INSTANTIATE_TEST_SUITE_P(
    DaqpTest, QuadraticProgramTest,
    ::testing::Combine(::testing::ValuesIn(quadratic_cost_form()),
                       ::testing::ValuesIn(linear_constraint_form()),
                       ::testing::ValuesIn(quadratic_problems())));

GTEST_TEST(DaqpSolverTest, UnitBallExample) {
  DaqpSolver solver;
  if (solver.available()) {
    TestQPonUnitBallExample(solver);
  }
}

GTEST_TEST(DaqpSolverTest, QuadraticCostVariableOrder) {
  DaqpSolver solver;
  if (solver.available()) {
    TestQuadraticCostVariableOrder(solver, kTol);
  }
}

GTEST_TEST(DaqpSolverTest, DuplicatedVariable) {
  DaqpSolver solver;
  if (solver.available()) {
    TestDuplicatedVariableQuadraticProgram(solver, kTol);
  }
}

GTEST_TEST(DaqpSolverTest, EqualityConstrainedQP1) {
  DaqpSolver solver;
  if (solver.available()) {
    TestEqualityConstrainedQP1(solver, kTol);
  }
}

GTEST_TEST(DaqpSolverTest, DualSolution1) {
  DaqpSolver solver;
  if (solver.available()) {
    // The expected dual in this example is rounded to 6 digits.
    TestQPDualSolution1(solver);
  }
}

GTEST_TEST(DaqpSolverTest, DualSolution2) {
  DaqpSolver solver;
  if (solver.available()) {
    TestQPDualSolution2(solver);
  }
}

GTEST_TEST(DaqpSolverTest, DualSolution3) {
  DaqpSolver solver;
  if (solver.available()) {
    // The sensitivity check uses a finite difference whose truncation error
    // (2.00001e-5 here) is just above the default tolerance.
    TestQPDualSolution3(solver, kTol, 3e-5);
  }
}

GTEST_TEST(DaqpSolverTest, EqualityConstrainedQPDualSolution1) {
  DaqpSolver solver;
  if (solver.available()) {
    TestEqualityConstrainedQPDualSolution1(solver);
  }
}

GTEST_TEST(DaqpSolverTest, EqualityConstrainedQPDualSolution2) {
  DaqpSolver solver;
  if (solver.available()) {
    TestEqualityConstrainedQPDualSolution2(solver);
  }
}

GTEST_TEST(DaqpSolverTest, NonconvexQP) {
  DaqpSolver solver;
  if (solver.available()) {
    TestNonconvexQP(solver, true);
  }
}

GTEST_TEST(DaqpSolverTest, UnconstrainedQP) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>("x");
  // (x₀ + 2)² + (x₁ + x₂ - 2)² + 1, with a positive semidefinite Hessian.
  prog.AddQuadraticCost(x(0) * x(0));
  prog.AddQuadraticCost((x(1) + x(2) - 2) * (x(1) + x(2) - 2));
  prog.AddLinearCost(4 * x(0) + 5);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_NEAR(result.GetSolution(x(0)), -2, kTol);
    EXPECT_NEAR(result.GetSolution(x(1)) + result.GetSolution(x(2)), 2, kTol);
    EXPECT_NEAR(result.get_optimal_cost(), 1, kTol);
    // Only the (inactive) variable bounds have multipliers.
    EXPECT_TRUE(
        CompareMatrices(result.get_solver_details<DaqpSolver>().multipliers,
                        Eigen::Vector3d::Zero()));
  }
}

GTEST_TEST(DaqpSolverTest, DifferentialIkCollisionConstraint) {
  // The clipped collision example from Drake's differential IK system test:
  // desired velocity (-8, 0.5), with collision constraint v_x >= -4.
  MathematicalProgram prog;
  const auto v = prog.NewContinuousVariables<2>("v");
  prog.AddQuadraticCost(2 * Eigen::Matrix2d::Identity(),
                        Eigen::Vector2d(16, -1), v);
  const auto collision = prog.AddLinearConstraint(
      Eigen::RowVector2d(1, 0), Vector1d(-4),
      Vector1d(std::numeric_limits<double>::infinity()), v);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(v), Eigen::Vector2d(-4, 0.5), kTol));
    EXPECT_NEAR(result.GetDualSolution(collision)(0), 8, kTol);
    // |v|² + 16 v₀ - v₁ at v = (-4, 0.5).
    EXPECT_NEAR(result.get_optimal_cost(), -48.25, kTol);
    const DaqpSolverDetails& details = result.get_solver_details<DaqpSolver>();
    EXPECT_EQ(details.exitflag, 1);
    EXPECT_GE(details.iterations, 1);
    EXPECT_GE(details.setup_time, 0);
    EXPECT_GE(details.solve_time, 0);
    // DAQP's own multipliers are negative at an active lower bound.
    EXPECT_TRUE(
        CompareMatrices(details.multipliers, Eigen::Vector3d(0, 0, -8), kTol));
  }
}

GTEST_TEST(DaqpSolverTest, OverlappingBoundingBoxes) {
  // Only the tightest bound owns the dual; the looser binding gets zero.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(x(0) * x(0));
  const auto loose = prog.AddBoundingBoxConstraint(0.5, 3, x);
  const auto tight = prog.AddBoundingBoxConstraint(1, 2, x);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_NEAR(result.GetSolution(x(0)), 1, kTol);
    EXPECT_NEAR(result.GetDualSolution(tight)(0), 2, kTol);
    EXPECT_NEAR(result.GetDualSolution(loose)(0), 0, kTol);
  }
}

GTEST_TEST(DaqpSolverTest, BoundingBoxDuplicatedVariable) {
  // One bounding box constraint that bounds x(0) twice. The effective bounds
  // are 3 ≤ x₀ ≤ 4 and 2 ≤ x₁ ≤ 5, so the solution is x = (3, 2).
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddQuadraticCost(x(0) * x(0) + x(1) * x(1));
  const auto bb_con = prog.AddBoundingBoxConstraint(
      Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(6, 5, 4),
      Vector3<symbolic::Variable>(x(0), x(1), x(0)));
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(x), Eigen::Vector2d(3, 2), kTol));
    // The gradient of the cost is 2x = (6, 4). The active lower bound on x(0)
    // is the third row; the looser first row gets zero.
    EXPECT_TRUE(CompareMatrices(result.GetDualSolution(bb_con),
                                Eigen::Vector3d(0, 4, 6), kTol));
  }
}

GTEST_TEST(DaqpSolverTest, EqualityBoundingBox) {
  // A bounding box with equal bounds is treated as an equality constraint.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>("x");
  prog.AddQuadraticCost(x(0) * x(0) + x(1) * x(1));
  const auto fixed = prog.AddBoundingBoxConstraint(1, 1, x(0));
  prog.AddLinearConstraint(x(0) + x(1) >= 3);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_TRUE(
        CompareMatrices(result.GetSolution(x), Eigen::Vector2d(1, 2), kTol));
    // L = x₀² + x₁² - λ(x₀ + x₁ - 3) - μ(x₀ - 1): λ = 4, μ = 2 - 4 = -2.
    EXPECT_NEAR(result.GetDualSolution(fixed)(0), -2, kTol);
  }
}

GTEST_TEST(DaqpSolverTest, Infeasible) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost(x(0) * x(0) + 2 * x(1) * x(1));
  prog.AddLinearConstraint(x(0) + 2 * x(1) == 2);
  prog.AddLinearConstraint(x(0) >= 1);
  prog.AddLinearConstraint(x(1) >= 2);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
    EXPECT_EQ(result.get_optimal_cost(),
              MathematicalProgram::kGlobalInfeasibleCost);
    EXPECT_EQ(result.get_solver_details<DaqpSolver>().multipliers.size(), 0);
  }
}

GTEST_TEST(DaqpSolverTest, InfeasibleBounds) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>();
  prog.AddQuadraticCost(x(0) * x(0));
  prog.AddBoundingBoxConstraint(2, 3, x);
  prog.AddBoundingBoxConstraint(-1, 1, x);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
  }
}

GTEST_TEST(DaqpSolverTest, Unbounded) {
  // With a positive semidefinite Hessian, DAQP's proximal-point iterations
  // do not detect unboundedness; they stop at the iteration limit instead.
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  prog.AddQuadraticCost(x(0) * x(0) + x(1));
  prog.SetSolverOption(DaqpSolver::id(), "iter_limit", 100);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(), SolutionResult::kIterationLimit);
  }
}

GTEST_TEST(DaqpSolverTest, SemidefiniteLeastSquaresCost) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddQuadraticCost((x(0) + x(1) - 1) * (x(0) + x(1) - 1));
  prog.AddBoundingBoxConstraint(0, 2, x);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_NEAR(result.GetSolution(x(0)) + result.GetSolution(x(1)), 1, 1e-6);
  }
}

// Minimizes |x - (1, 1, 1)|² subject to x ≤ 0, which needs one active-set
// iteration per variable.
void AddThreeActiveBoundsProgram(MathematicalProgram* prog) {
  const auto x = prog->NewContinuousVariables<3>();
  prog->AddQuadraticCost(2 * Eigen::Matrix3d::Identity(),
                         -2 * Eigen::Vector3d::Ones(), 3, x);
  prog->AddBoundingBoxConstraint(-10, 0, x);
}

GTEST_TEST(DaqpSolverTest, SolverOptions) {
  MathematicalProgram prog;
  AddThreeActiveBoundsProgram(&prog);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    const int iterations = result.get_solver_details<DaqpSolver>().iterations;
    ASSERT_GE(iterations, 2);

    SolverOptions options;
    options.SetOption(DaqpSolver::id(), "iter_limit", iterations - 1);
    const auto limited = solver.Solve(prog, std::nullopt, options);
    EXPECT_EQ(limited.get_solution_result(), SolutionResult::kIterationLimit);
    EXPECT_EQ(limited.get_solver_details<DaqpSolver>().exitflag, -4);

    // Floating-point options are accepted.
    SolverOptions tolerances;
    tolerances.SetOption(DaqpSolver::id(), "primal_tol", 1e-10);
    tolerances.SetOption(DaqpSolver::id(), "dual_tol", 1e-10);
    EXPECT_TRUE(solver.Solve(prog, std::nullopt, tolerances).is_success());

    // Common options have no effect, and don't throw.
    SolverOptions common;
    common.SetOption(CommonSolverOption::kPrintToConsole, 1);
    common.SetOption(CommonSolverOption::kMaxThreads, 1);
    EXPECT_TRUE(solver.Solve(prog, std::nullopt, common).is_success());

    // Options set on the program are used too.
    prog.SetSolverOption(DaqpSolver::id(), "iter_limit", iterations - 1);
    EXPECT_EQ(solver.Solve(prog).get_solution_result(),
              SolutionResult::kIterationLimit);
  }
}

GTEST_TEST(DaqpSolverTest, UnknownOption) {
  MathematicalProgram prog;
  AddThreeActiveBoundsProgram(&prog);
  DaqpSolver solver;
  if (solver.available()) {
    SolverOptions options;
    options.SetOption(DaqpSolver::id(), "max_iter", 10);
    DRAKE_EXPECT_THROWS_MESSAGE(solver.Solve(prog, std::nullopt, options),
                                ".*not recognized.*max_iter.*");
  }
}

GTEST_TEST(DaqpSolverTest, DetailsAreResetBetweenSolves) {
  MathematicalProgram feasible;
  AddThreeActiveBoundsProgram(&feasible);
  MathematicalProgram infeasible;
  const auto x = infeasible.NewContinuousVariables<1>();
  infeasible.AddQuadraticCost(x(0) * x(0));
  infeasible.AddLinearConstraint(x(0) >= 1);
  infeasible.AddLinearConstraint(x(0) <= 0);
  DaqpSolver solver;
  if (solver.available()) {
    MathematicalProgramResult result;
    solver.Solve(feasible, std::nullopt, std::nullopt, &result);
    ASSERT_TRUE(result.is_success());
    EXPECT_EQ(result.get_solver_details<DaqpSolver>().multipliers.size(), 3);
    solver.Solve(infeasible, std::nullopt, std::nullopt, &result);
    EXPECT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solver_details<DaqpSolver>().multipliers.size(), 0);
  }
}

GTEST_TEST(DaqpSolverTest, VariableScalingIsIgnored) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  prog.AddLinearConstraint(2 * x(0) - 2 * x(1) == 2);
  prog.AddQuadraticCost((x(0) + 1) * (x(0) + 1) + (x(1) + 1) * (x(1) + 1));
  prog.SetVariableScaling(x(0), 100);
  DaqpSolver solver;
  if (solver.available()) {
    const auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    EXPECT_TRUE(CompareMatrices(result.GetSolution(x),
                                Eigen::Vector2d(-0.5, -1.5), kTol));
  }
}

GTEST_TEST(DaqpSolverTest, ProgramAttributesGood) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(x(0) * x(0));
  EXPECT_TRUE(DaqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_EQ(DaqpSolver::UnsatisfiedProgramAttributes(prog), "");
}

GTEST_TEST(DaqpSolverTest, ProgramAttributesBad) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddCost(x(0) * x(0) * x(0));
  EXPECT_FALSE(DaqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_THAT(DaqpSolver::UnsatisfiedProgramAttributes(prog),
              HasSubstr("GenericCost was declared"));
}

GTEST_TEST(DaqpSolverTest, ProgramAttributesMisfit) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddLinearCost(4 * x(0) + 5);
  EXPECT_FALSE(DaqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_THAT(DaqpSolver::UnsatisfiedProgramAttributes(prog),
              HasSubstr("QuadraticCost is required"));
}

GTEST_TEST(DaqpSolverTest, ProgramAttributesNonconvex) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<1>("x");
  prog.AddQuadraticCost(-x(0) * x(0), false);
  EXPECT_FALSE(DaqpSolver::ProgramAttributesSatisfied(prog));
  EXPECT_THAT(DaqpSolver::UnsatisfiedProgramAttributes(prog),
              HasSubstr("non-convex"));
}

GTEST_TEST(DaqpSolverTest, SolverRegistry) {
  const auto solver = MakeSolver(DaqpSolver::id());
  EXPECT_EQ(solver->solver_id(), DaqpSolver::id());
  EXPECT_EQ(solver->solver_id().name(), "DAQP");
  EXPECT_TRUE(GetKnownSolvers().contains(DaqpSolver::id()));
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
