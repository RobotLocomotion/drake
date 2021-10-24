#include "drake/solvers/ibex_solver.h"

#include <math.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"

namespace drake {
namespace solvers {
namespace test {
namespace {

using std::exp;
using std::make_shared;
using std::pow;
using std::shared_ptr;
using std::sqrt;

class IbexSolverTest : public ::testing::Test {
 protected:
  MathematicalProgram prog_;
  VectorXDecisionVariable x_;
  IbexSolver solver_;
};

// Reproduced from
// https://github.com/ibex-team/ibex-lib/blob/master/benchs/optim/easy/ex3_1_3.bch
TEST_F(IbexSolverTest, IbexEasyEx3_1_3) {
  x_ = prog_.NewContinuousVariables(6, "x");
  Vector6d lb, ub, x_d, Q_diag;
  lb << 0, 0, 1, 0, 1, 0;
  ub << 1e8, 1e8, 5, 6, 5, 10;
  prog_.AddBoundingBoxConstraint(lb, ub, x_);
  prog_.AddCost(-25 * pow(x_[0] - 2, 2) - pow(x_[1] - 2, 2) -
                pow(x_[2] - 1, 2) - pow(x_[3] - 4, 2) - pow(x_[4] - 1, 2) -
                pow(x_[5] - 4, 2));
  prog_.AddConstraint(pow(x_[2] - 3, 2) + x_[3] >= 4);
  prog_.AddConstraint(pow(x_[4] - 3, 2) + x_[5] >= 4);
  prog_.AddConstraint(x_[0] - 3 * x_[1] <= 2);
  prog_.AddConstraint(-x_[0] + x_[1] <= 2);
  prog_.AddConstraint(x_[0] + x_[1] <= 6);
  prog_.AddConstraint(x_[0] + x_[1] >= 2);
  if (solver_.available()) {
    SolverOptions solver_options;
    // Print to console. We can only test this doesn't cause any runtime error.
    // We can't test if the logging message is actually printed to the console.
    solver_options.SetOption(CommonSolverOption::kPrintToConsole, 1);
    auto result = solver_.Solve(prog_, {}, solver_options);
    ASSERT_TRUE(result.is_success());
    // f* in  [-310.31,-310]
    //  (best bound)
    //
    // x* = (5 ; 1 ; 5 ; 1e-09 ; 5 ; 10)
    //  (best feasible point)
    EXPECT_TRUE(result.get_optimal_cost() <= -310.0);
    EXPECT_TRUE(-310.31 <= result.get_optimal_cost());

    const auto x_val = result.GetSolution(prog_.decision_variables());
    Vector6d expected_val;
    expected_val << 5, 1, 5, 1e-09, 5, 10;
    EXPECT_NEAR((x_val - expected_val).norm(), 0.0, 1e-6);
  }
}

TEST_F(IbexSolverTest, OnlyConstraints) {
  x_ = prog_.NewContinuousVariables(6, "x");
  Vector6d lb, ub, x_d, Q_diag;
  lb << 0, 0, 1, 0, 1, 0;
  ub << 1e8, 1e8, 5, 6, 5, 10;
  prog_.AddBoundingBoxConstraint(lb, ub, x_);
  prog_.AddConstraint(pow(x_[2] - 3, 2) + x_[3] >= 4);
  prog_.AddConstraint(pow(x_[4] - 3, 2) + x_[5] >= 4);
  prog_.AddConstraint(x_[0] - 3 * x_[1] <= 2);
  prog_.AddConstraint(-x_[0] + x_[1] <= 2);
  prog_.AddConstraint(x_[0] + x_[1] <= 6);
  prog_.AddConstraint(x_[0] + x_[1] >= 2);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_);
    ASSERT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(prog_.decision_variables());
    EXPECT_GE(pow(x_val[2] - 3, 2) + x_val[3], 4);
    EXPECT_GE(pow(x_val[4] - 3, 2) + x_val[5], 4);
    EXPECT_LE(x_val[0] - 3 * x_val[1], 2);
    EXPECT_LE(-x_val[0] + x_val[1], 2);
    EXPECT_LE(x_val[0] + x_val[1], 6);
    EXPECT_GE(x_val[0] + x_val[1], 2);
  }
}

TEST_F(IbexSolverTest, GenericCost) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddBoundingBoxConstraint(1, 2, x0);
  prog_.AddBoundingBoxConstraint(1, 2, x1);
  prog_.AddBoundingBoxConstraint(1, 2, x2);
  const shared_ptr<Cost> generic_cost =
      make_shared<test::GenericTrivialCost1>();
  prog_.AddCost(Binding<Cost>(generic_cost, x_));
  EXPECT_FALSE(prog_.generic_costs().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_);
    ASSERT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(prog_.decision_variables());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    EXPECT_NEAR(v0, M_SQRT2, 1e-8);
    EXPECT_NEAR(v1, 1, 1e-8);
    EXPECT_NEAR(v2, 1, 1e-8);
  }
}

TEST_F(IbexSolverTest, GenericConstraint) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddBoundingBoxConstraint(-10, 10, x0);
  prog_.AddBoundingBoxConstraint(-10, 10, x1);
  prog_.AddBoundingBoxConstraint(-10, 10, x2);
  const shared_ptr<Constraint> generic_constraint =
      make_shared<test::GenericTrivialConstraint1>();
  prog_.AddConstraint(Binding<Constraint>(generic_constraint, x_));
  EXPECT_FALSE(prog_.generic_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_);
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
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

TEST_F(IbexSolverTest, ExponentialConeConstraint) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddConstraint(x0, -5, 5);
  prog_.AddConstraint(x1, -5, 5);
  prog_.AddConstraint(x2, -5, 5);
  Eigen::SparseMatrix<double> A(3, 3);
  A.setIdentity();
  prog_.AddExponentialConeConstraint(A, Eigen::Vector3d(1, 2, 3), x_);
  EXPECT_FALSE(prog_.exponential_cone_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    EXPECT_LE(0, 1 + v0 - ((2 + v1) * exp(((3 + v2) / (2 + v1)))));
    EXPECT_LE(0, 2 + v1);
  }
}

TEST_F(IbexSolverTest, LinearEqualityConstraint) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddConstraint(x0, -5, 5);
  prog_.AddConstraint(x1, -5, 5);
  prog_.AddConstraint(x2, -5, 5);
  prog_.AddConstraint(2 * x0 - 3 * x1 + 4 * x2 == 1);
  EXPECT_FALSE(prog_.linear_equality_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    EXPECT_NEAR(1, 2 * v0 - 3 * v1 + 4 * v2, 1e-8);
  }
}

TEST_F(IbexSolverTest, LorentzConeConstraint) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddConstraint(x0, -5, 5);
  prog_.AddConstraint(x1, -5, 5);
  prog_.AddConstraint(x2, 0, 5);
  prog_.AddLorentzConeConstraint(Vector3<symbolic::Expression>(x2, x0, x1));
  EXPECT_FALSE(prog_.lorentz_cone_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    EXPECT_LE(0, v2 - sqrt((pow(v0, 2) + pow(v1, 2))));
  }
}

TEST_F(IbexSolverTest, RotatedLorentzConeConstraint) {
  x_ = prog_.NewContinuousVariables(3, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  const auto& x2{x_(2)};
  prog_.AddConstraint(x0, -1, 1);
  prog_.AddConstraint(x1, -1, 1);
  prog_.AddConstraint(x2, -1, 1);
  prog_.AddRotatedLorentzConeConstraint(
      Vector4<symbolic::Expression>(x0 + x1, x1 + x2, +x0, +x1));
  EXPECT_FALSE(prog_.rotated_lorentz_cone_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    EXPECT_LE(0, v0 + v1);
    EXPECT_LE(0, v1 + v2);
    EXPECT_LE(0, (v0 + v1) * (v1 + v2) - pow(v0, 2) - pow(v1, 2));
  }
}

TEST_F(IbexSolverTest, LinearComplementarityConstraint) {
  x_ = prog_.NewContinuousVariables(2, "x");
  const auto& x0{x_(0)};
  const auto& x1{x_(1)};
  prog_.AddConstraint(x0, -1, 1);
  prog_.AddConstraint(x1, -1, 1);
  prog_.AddLinearComplementarityConstraint(Eigen::Matrix2d::Identity(),
                                           Eigen::Vector2d::Ones(), x_);
  EXPECT_FALSE(prog_.linear_complementarity_constraints().empty());
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    EXPECT_NEAR(v0 * (1 + v0) + v1 * (1 + v1), 0, 1e-8);
    EXPECT_GE(v0, 0);
    EXPECT_GE(v1, 0);
    EXPECT_GE(1 + v0, 0);
    EXPECT_GE(1 + v1, 0);
  }
}

TEST_F(IbexSolverTest, WrongOptions) {
  if (solver_.available()) {
    {
      // Test a double option.
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "eps_H" /* typo of eps_h */, 0.1);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: unsupported option 'eps_H' with double value '0.1'.");
    }

    {
      // Test an int option.
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "rigoor" /* typo of rigor */, 1);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: unsupported option 'rigoor' with int value '1'.");
    }

    // 'trace' should be in {0, 1, 2}.
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "trace", -1);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: cannot set IBEX Solver option 'trace' to value '-1'. It "
          "should be 0, 1, or, 2.");
    }
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "trace", 3);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: cannot set IBEX Solver option 'trace' to value '3'. It "
          "should be 0, 1, or, 2.");
    }

    // No string options.
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "trace", "1" /* should be int */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: unsupported option 'trace' with string value '1'.");
    }
  }
}

TEST_F(IbexSolverTest, InvalidValues) {
  if (solver_.available()) {
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "rel_eps_f",
                               -3.1 /* should be positive */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: 'rel_eps_f' option should have a "
          "positive value but '-3.1' was provided.");
    }
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "abs_eps_f",
                               -3.1 /* should be positive */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: 'abs_eps_f' option should have a "
          "positive value but '-3.1' was provided.");
    }
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "eps_h",
                               -3.1 /* should be positive */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: 'eps_h' option should have a positive "
          "value but '-3.1' was provided.");
    }
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "eps_x",
                               -3.1 /* should be positive */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: 'eps_x' option should have a positive "
          "value but '-3.1' was provided.");
    }
    {
      SolverOptions solver_options;
      solver_options.SetOption(solver_.id(), "timeout",
                               -3.1 /* should be non-negative */);
      DRAKE_EXPECT_THROWS_MESSAGE(
          solver_.Solve(prog_, {}, solver_options),
          "IbexSolver: 'timeout' option should have a non-negative "
          "value but '-3.1' was provided.");
    }
  }
}

class IbexSolverOptionTest1 : public ::testing::Test {
 protected:
  void SetUp() override {
    x_ = prog_.NewContinuousVariables(3, "x");
    const auto& x0{x_(0)};
    const auto& x1{x_(1)};
    const auto& x2{x_(2)};
    prog_.AddConstraint(x0, -5, 5);
    prog_.AddConstraint(x1, -5, 5);
    prog_.AddConstraint(x2, -5, 5);
    prog_.AddConstraint(2 * x0 - 3 * x1 + 4 * x2 <= 1.0 + 1e-10);
    prog_.AddConstraint(2 * x0 - 3 * x1 + 4 * x2 >= 1.0 - 1e-10);
    prog_.AddCost(x_(1) + x_(2));
  }

  MathematicalProgram prog_;
  VectorXDecisionVariable x_;
  IbexSolver solver_;
};

// Checks if using two different options causes IBEX to generate different
// results. Note that it requires that IBEX solves the problem successfully with
// option1.
::testing::AssertionResult CheckIfIbexBehavesDifferently(
    const IbexSolver& solver, const MathematicalProgram& prog,
    const SolverOptions& options1, const SolverOptions& options2) {
  if (!solver.available()) {
    return ::testing::AssertionSuccess();
  }
  // Solve with options1.
  const auto result1 = solver.Solve(prog, {}, options1);
  if (!result1.is_success()) {
    return ::testing::AssertionFailure() << "IBEX failed with options1.";
  }

  // Solve with options2.
  const auto result2 = solver.Solve(prog, {}, options2);
  if (!result2.is_success()) {
    // IBEX failed with option2 => OK.
    return ::testing::AssertionSuccess();
  }

  // result1 and result2 should be different.
  const auto x_val_1 = result1.GetSolution(prog.decision_variables());
  const auto x_val_2 = result2.GetSolution(prog.decision_variables());
  if (x_val_1 != x_val_2) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "Result 1 and Result 2 are the same " << x_val_1 << ".";
  }
}

TEST_F(IbexSolverOptionTest1, RelEpsF) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "rel_eps_f", 1.0);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

TEST_F(IbexSolverOptionTest1, AbsEpsF) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "abs_eps_f", 1.0);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

TEST_F(IbexSolverOptionTest1, RandomSeed) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "random_seed", 7.0);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

TEST_F(IbexSolverOptionTest1, EpsX) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "eps_x", 1.0);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

// The following Trace0, Trace1, and Trace2 test cases run the solver with
// different 'trace' option values. We cannot check messages written to the
// console; we're simply testing exception-free execution.
TEST_F(IbexSolverOptionTest1, Trace0) {
  if (solver_.available()) {
    SolverOptions solver_options;
    solver_options.SetOption(solver_.id(), "trace", 0);
    EXPECT_NO_THROW(solver_.Solve(prog_, {}, solver_options));
  }
}

TEST_F(IbexSolverOptionTest1, Trace1) {
  if (solver_.available()) {
    SolverOptions solver_options;
    solver_options.SetOption(solver_.id(), "trace", 1);
    EXPECT_NO_THROW(solver_.Solve(prog_, {}, solver_options));
  }
}

TEST_F(IbexSolverOptionTest1, Trace2) {
  if (solver_.available()) {
    SolverOptions solver_options;
    solver_options.SetOption(solver_.id(), "trace", 2);
    EXPECT_NO_THROW(solver_.Solve(prog_, {}, solver_options));
  }
}

TEST_F(IbexSolverOptionTest1, Timeout) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "timeout", 1e-30);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

// We introduce another program to show that 'rigor' and 'eps_h' options change
// the behavior of IBEX.
class IbexSolverOptionTest2 : public ::testing::Test {
 protected:
  void SetUp() override {
    // From
    // https://github.com/ibex-team/ibex-lib/blob/master/benchs/optim/easy/wall.bch
    x_ = prog_.NewContinuousVariables(6, "x");
    const auto& x1{x_(0)};
    const auto& x2{x_(1)};
    const auto& x3{x_(2)};
    const auto& x4{x_(3)};
    const auto& x5{x_(4)};
    const auto& x6{x_(5)};

    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x1);
    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x2);
    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x3);
    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x4);
    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x5);
    prog_.AddBoundingBoxConstraint(-1.0e-8, 1.0e+8, x6);

    prog_.AddConstraint(x1 * x2 - 1 == 0);
    prog_.AddConstraint(x3 / x1 / x4 - 4.8 == 0);
    prog_.AddConstraint(x5 / x2 / x6 - 0.98 == 0);
    prog_.AddConstraint(x6 * x4 - 1 == 0);
    prog_.AddConstraint(x1 - x2 + 1.e-7 * x3 - 1.e-5 * x5 == 0);
    prog_.AddConstraint(
        2 * x1 - 2 * x2 + 1.e-7 * x3 - 0.01 * x4 - 1.e-5 * x5 + 0.01 * x6 == 0);

    prog_.AddCost(x1);
  }

  MathematicalProgram prog_;
  VectorXDecisionVariable x_;
  IbexSolver solver_;
};

TEST_F(IbexSolverOptionTest2, EpsH) {
  const SolverOptions options1;  // defaults.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "eps_h", 1e-3);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

TEST_F(IbexSolverOptionTest2, Rigor) {
  const SolverOptions options1;  // default, rigor = 0.
  SolverOptions options2;
  options2.SetOption(solver_.id(), "rigor", 1);
  EXPECT_TRUE(
      CheckIfIbexBehavesDifferently(solver_, prog_, options1, options2));
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
