#include "drake/solvers/ibex_solver.h"

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"

namespace drake {
namespace solvers {
namespace test {
namespace {

using std::make_shared;
using std::pow;
using std::shared_ptr;

// Reproduced from
// https://github.com/ibex-team/ibex-lib/blob/master/benchs/optim/easy/ex3_1_3.bch
GTEST_TEST(IbexSolverTest, IbexEasyEx3_1_3) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(6, "x");
  Vector6d lb, ub, x_d, Q_diag;
  lb << 0, 0, 1, 0, 1, 0;
  ub << 1e8, 1e8, 5, 6, 5, 10;
  prog.AddBoundingBoxConstraint(lb, ub, x);
  prog.AddCost(-25 * pow(x[0] - 2, 2) - pow(x[1] - 2, 2) - pow(x[2] - 1, 2) -
               pow(x[3] - 4, 2) - pow(x[4] - 1, 2) - pow(x[5] - 4, 2));
  prog.AddConstraint(pow(x[2] - 3, 2) + x[3] >= 4);
  prog.AddConstraint(pow(x[4] - 3, 2) + x[5] >= 4);
  prog.AddConstraint(x[0] - 3 * x[1] <= 2);
  prog.AddConstraint(-x[0] + x[1] <= 2);
  prog.AddConstraint(x[0] + x[1] <= 6);
  prog.AddConstraint(x[0] + x[1] >= 2);
  IbexSolver ibex_solver;
  if (ibex_solver.available()) {
    auto result = ibex_solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    // f* in  [-310.31,-310]
    //  (best bound)
    //
    // x* = (5 ; 1 ; 5 ; 1e-09 ; 5 ; 10)
    //  (best feasible point)
    EXPECT_TRUE(result.get_optimal_cost() <= -310.0);
    EXPECT_TRUE(-310.31 <= result.get_optimal_cost());

    const auto x_val = result.GetSolution(prog.decision_variables());
    Vector6d expected_val;
    expected_val << 5, 1, 5, 1e-09, 5, 10;
    EXPECT_NEAR((x_val - expected_val).norm(), 0.0, 1e-6);
  }
}

GTEST_TEST(IbexSolverTest, OnlyConstraints) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(6, "x");
  Vector6d lb, ub, x_d, Q_diag;
  lb << 0, 0, 1, 0, 1, 0;
  ub << 1e8, 1e8, 5, 6, 5, 10;
  prog.AddBoundingBoxConstraint(lb, ub, x);

  prog.AddConstraint(pow(x[2] - 3, 2) + x[3] >= 4);
  prog.AddConstraint(pow(x[4] - 3, 2) + x[5] >= 4);
  prog.AddConstraint(x[0] - 3 * x[1] <= 2);
  prog.AddConstraint(-x[0] + x[1] <= 2);
  prog.AddConstraint(x[0] + x[1] <= 6);
  prog.AddConstraint(x[0] + x[1] >= 2);

  IbexSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());

    const auto x_val = result.GetSolution(prog.decision_variables());
    EXPECT_GE(pow(x_val[2] - 3, 2) + x_val[3], 4);
    EXPECT_GE(pow(x_val[4] - 3, 2) + x_val[5], 4);
    EXPECT_LE(x_val[0] - 3 * x_val[1], 2);
    EXPECT_LE(-x_val[0] + x_val[1], 2);
    EXPECT_LE(x_val[0] + x_val[1], 6);
    EXPECT_GE(x_val[0] + x_val[1], 2);
  }
}

GTEST_TEST(IbexSolverTest, GenericCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  prog.AddBoundingBoxConstraint(1, 2, x(0));
  prog.AddBoundingBoxConstraint(1, 2, x(1));
  prog.AddBoundingBoxConstraint(1, 2, x(2));
  const shared_ptr<Cost> generic_cost =
      make_shared<test::GenericTrivialCost1>();
  prog.AddCost(Binding<Cost>(generic_cost, x));

  IbexSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(prog.decision_variables());
    const double v0{result.GetSolution(x(0))};
    const double v1{result.GetSolution(x(1))};
    const double v2{result.GetSolution(x(2))};
    EXPECT_NEAR(v0, 1.413078079, 1e-8);
    EXPECT_NEAR(v1, 1, 1e-8);
    EXPECT_NEAR(v2, 1, 1e-8);
  }
}

GTEST_TEST(IbexSolverTest, GenericConstraint) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  prog.AddBoundingBoxConstraint(-10, 10, x(0));
  prog.AddBoundingBoxConstraint(-10, 10, x(1));
  prog.AddBoundingBoxConstraint(-10, 10, x(2));
  const shared_ptr<Constraint> generic_constraint =
      make_shared<test::GenericTrivialConstraint1>();
  prog.AddConstraint(Binding<Constraint>(generic_constraint, x));

  IbexSolver solver;
  if (solver.available()) {
    auto result = solver.Solve(prog);
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x(0))};
    const double v1{result.GetSolution(x(1))};
    const double v2{result.GetSolution(x(2))};
    EXPECT_LE(-10, v0);
    EXPECT_LE(v0, 10);
    EXPECT_LE(-10, v1);
    EXPECT_LE(v1, 10);
    EXPECT_LE(-10, v2);
    EXPECT_LE(v2, 10);
    EXPECT_LE(-1, v0 * v1 + v2 / v0 * 2);
    EXPECT_LE(v0 * v1 + v2 / v0 * 2, 2);
    EXPECT_LE(-2, v1 * v2 - v0);
    EXPECT_LE(v1 * v2 - v0, 1);
  }
}

}  // namespace

}  // namespace test
}  // namespace solvers
}  // namespace drake
