#include "drake/solvers/ibex_solver.h"

#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"

namespace drake {
namespace solvers {
namespace {

using std::pow;

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

}  // namespace

}  // namespace solvers
}  // namespace drake
