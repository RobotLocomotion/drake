#include "drake/solvers/ibex_solver.h"

#include <cmath>
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
    auto result = solver_.Solve(prog_);
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
    EXPECT_NEAR(v0, 1.413078079, 1e-8);
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

}  // namespace

}  // namespace test
}  // namespace solvers
}  // namespace drake
