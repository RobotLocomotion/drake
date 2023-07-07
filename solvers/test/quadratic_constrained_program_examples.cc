#include "drake/solvers/test/quadratic_constrained_program_examples.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace solvers {
namespace test {
const double kInf = std::numeric_limits<double>::infinity();

void TestEllipsoid1(const SolverInterface& solver,
                    const std::optional<SolverOptions>& solver_options,
                    double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const Eigen::Matrix2d Q = Eigen::Vector2d(8, 18).asDiagonal();
  auto quadratic_con =
      prog.AddQuadraticConstraint(Q, Eigen::Vector2d::Zero(), -kInf, 1, x);
  prog.AddLinearCost(Eigen::Vector2d(1, 1), x);
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, solver_options, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Vector2d x_sol = result.GetSolution(x);
  EXPECT_NEAR(result.get_optimal_cost(), -std::sqrt(13) / 6, tol);
  const Eigen::Vector2d x_expected(-3 * std::sqrt(13) / 26,
                                   -4 * std::sqrt(13) / 78);
  EXPECT_TRUE(CompareMatrices(x_sol, x_expected, tol));
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(quadratic_con),
                              -Vector1d(std::sqrt(13) / 12), tol));
}

void TestEllipsoid2(const SolverInterface& solver,
                    const std::optional<SolverOptions>& solver_options,
                    double tol) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // We intentially wirte this constraint as -x0² - x0*x1 - x1² >= -1 to test
  // the quadratic constraint with a lower bound.
  const auto quadratic_constraint1 = prog.AddQuadraticConstraint(
      -x(0) * x(0) - x(0) * x(1) - x(1) * x(1), -1, kInf);
  const auto quadratic_constraint2 = prog.AddQuadraticConstraint(
      x(0) * x(0) + 2 * x(0) * x(1) + 4 * x(1) * x(1), -kInf, 9);
  const auto linear_constraint = prog.AddLinearConstraint(x(0) + x(1) <= 0);
  prog.AddLinearCost(x(0) + x(1));
  MathematicalProgramResult result;
  solver.Solve(prog, std::nullopt, solver_options, &result);
  EXPECT_TRUE(result.is_success());
  const Eigen::Vector2d x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(
      x_sol, Eigen::Vector2d(-std::sqrt(3) / 3, -std::sqrt(3) / 3), tol));
  EXPECT_NEAR(result.get_optimal_cost(), -2 * std::sqrt(3) / 3, tol);
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(quadratic_constraint1),
                              Vector1d(std::sqrt(3) / 3), tol));
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(quadratic_constraint2),
                              Vector1d(0), tol));
  EXPECT_TRUE(CompareMatrices(result.GetDualSolution(linear_constraint),
                              Vector1d(0), tol));
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
