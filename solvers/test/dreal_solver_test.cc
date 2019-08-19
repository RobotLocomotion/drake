#include "drake/solvers/dreal_solver.h"

#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/generic_trivial_constraints.h"
#include "drake/solvers/test/generic_trivial_costs.h"

namespace drake {
namespace solvers {
namespace {

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::Variables;

using std::logic_error;
using std::make_shared;
using std::shared_ptr;
using std::vector;

class DrealSolverTest : public ::testing::Test {
 protected:
  void SetUp() override { xvec_ = prog_.NewContinuousVariables(4, "x"); }

  // Continuous variables.
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};

  // Integer variables.
  const Variable i_{"i", Variable::Type::INTEGER};
  const Variable j_{"j", Variable::Type::INTEGER};

  // Binary variables.
  const Variable binary1_{"binary1", Variable::Type::BINARY};
  const Variable binary2_{"binary2", Variable::Type::BINARY};

  // Boolean variables.
  const Variable b1_{"b1", Variable::Type::BOOLEAN};
  const Variable b2_{"b2", Variable::Type::BOOLEAN};
  const Variable b3_{"b3", Variable::Type::BOOLEAN};

  const double delta_{0.001};
  MathematicalProgram prog_;
  VectorXDecisionVariable xvec_;
  DrealSolver solver_;
};

TEST_F(DrealSolverTest, Interval) {
  const double low{-10.0};
  const double high{10.0};
  DrealSolver::Interval i{low, high};

  EXPECT_EQ(i.low(), low);
  EXPECT_EQ(i.high(), high);
  EXPECT_EQ(i.mid(), 0.0);
  EXPECT_EQ(i.diam(), 20.0);
}

TEST_F(DrealSolverTest, Available) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        Expression{0.0} > Expression{1.0}, delta_);
    ASSERT_FALSE(result);
  }
}

// 0.0 > 1.0 is trivially UNSAT.
TEST_F(DrealSolverTest, CheckSatisfiabilityTrivialUnsat) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        Expression{0.0} > Expression{1.0}, delta_);
    ASSERT_FALSE(result);
  }
}

// 1.0 > 0.0 is trivially SAT.
TEST_F(DrealSolverTest, CheckSatisfiabilityTrivialSat) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        Expression{1.0} > Expression{0.0}, delta_);
    ASSERT_TRUE(result);
    // The result is an empty box.
    EXPECT_EQ(result->size(), 0.0);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityConjunction) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(b1_ && !b2_ && !b3_, delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    // We should have a point solution (i.e. lower-bound == upper-bound).
    EXPECT_EQ(solution.at(b1_).diam(), 0);
    EXPECT_EQ(solution.at(b2_).diam(), 0);
    EXPECT_EQ(solution.at(b3_).diam(), 0);
    const double v1{solution.at(b1_).mid()};
    const double v2{solution.at(b2_).mid()};
    const double v3{solution.at(b3_).mid()};
    // Should be either 1.0 (representing True) or 0.0 (False).
    EXPECT_TRUE(v1 == 1.0 || v1 == 0.0);
    EXPECT_TRUE(v2 == 1.0 || v2 == 0.0);
    EXPECT_TRUE(v3 == 1.0 || v3 == 0.0);
    EXPECT_TRUE(v1 && !v2 && !v3);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityDisjunction) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(b1_ || !b2_ || b3_, delta_);
    const DrealSolver::IntervalBox& solution{*result};
    // We should have a point solution (i.e. lower-bound == upper-bound).
    EXPECT_EQ(solution.at(b1_).diam(), 0);
    EXPECT_EQ(solution.at(b2_).diam(), 0);
    EXPECT_EQ(solution.at(b3_).diam(), 0);
    const double v1{solution.at(b1_).mid()};
    const double v2{solution.at(b2_).mid()};
    const double v3{solution.at(b3_).mid()};
    // Should be either 1.0 (representing True) or 0.0 (False).
    EXPECT_TRUE(v1 == 1.0 || v1 == 0.0);
    EXPECT_TRUE(v2 == 1.0 || v2 == 0.0);
    EXPECT_TRUE(v3 == 1.0 || v3 == 0.0);
    EXPECT_TRUE(v1 || !v2 || v3);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityLinearReal) {
  const Formula f1{0 <= x_ && x_ <= 5};
  const Formula f2{0 <= y_ && y_ <= 5};
  const Formula f3{2 * x_ + 3 * y_ == 5 && -3 * x_ + 4 * y_ == 6};

  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f1 && f2 && f3,
                                                         delta_);
    ASSERT_TRUE(result);

    const DrealSolver::IntervalBox& solution{*result};
    const double expected_x{2.0 / 17.0};
    const double expected_y{27.0 / 17.0};
    EXPECT_NEAR(solution.at(x_).mid(), expected_x, delta_);
    EXPECT_NEAR(solution.at(y_).mid(), expected_y, delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityLinearInteger) {
  const Formula f1{0 <= i_ && i_ <= 5};
  const Formula f2{0 <= j_ && j_ <= 5};
  const Formula f3{2 * i_ + 3 * j_ == 5 && -3 * i_ + 4 * j_ == 6};
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f1 && f2 && f3,
                                                         delta_);
    // Note that this has the same constraint as the previous,
    // CheckSatisfiabilityLinearReal test. However, the domain constraint, i,j ∈
    // Z, makes the problem unsatisfiable.
    EXPECT_FALSE(result);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityBinaryVariables) {
  const Formula f{2 * binary1_ + 3 * binary2_ == 0};
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f, delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    // Because of the domain constraints, the only solution is {binary1 ↦ 0,
    // binary2 ↦ 0}.
    EXPECT_EQ(solution.at(binary1_).mid(), 0.0);
    EXPECT_EQ(solution.at(binary2_).mid(), 0.0);
    // They are points.
    EXPECT_EQ(solution.at(binary1_).diam(), 0.0);
    EXPECT_EQ(solution.at(binary2_).diam(), 0.0);
  }
}

// Tests CheckSatisfiability (δ-SAT case).
TEST_F(DrealSolverTest, CheckSatisfiabilityDeltaSat) {
  // Find a model satisfying the following constraints:
  //     0 ≤ x ≤ 5
  //     0 ≤ y ≤ 5
  //     0 ≤ z ≤ 5
  //     2x² + y = z
  const Formula f1{0 <= x_ && x_ <= 5};
  const Formula f2{0 <= y_ && y_ <= 5};
  const Formula f3{0 <= z_ && z_ <= 5};
  const Formula f4{2 * x_ * x_ + y_ == z_};

  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(f1 && f2 && f3 && f4, delta_);
    ASSERT_TRUE(result);

    const double x{result->at(x_).mid()};
    const double y{result->at(y_).mid()};
    const double z{result->at(z_).mid()};
    EXPECT_TRUE(0 <= x && x <= 5);
    EXPECT_TRUE(0 <= y && y <= 5);
    EXPECT_TRUE(0 <= z && z <= 5);
    EXPECT_NEAR(2 * x * x + y, z, delta_);
  }
}

// Tests CheckSatisfiability (UNSAT case).
TEST_F(DrealSolverTest, CheckSatisfiabilityUnsat) {
  // Find a model satisfying the following constraints:
  //     2x² + 6x + 5 < 0
  //     -10 ≤ x ≤ 10
  const Formula f1{2 * x_ * x_ + 6 * x_ + 5 < 0};
  const Formula f2{-10 <= x_ && x_ <= 10};

  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f1 && f2, delta_);
    EXPECT_FALSE(result);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityNonlinear) {
  // Find a model satisfying the following constraints:
  //     x = 4
  //     y >= abs(-x²)
  //     sqrt(y) / 4 ≠ 1
  //     25 > y
  const Formula f1{x_ == 4};
  const Formula f2{y_ >= abs(-x_ * x_)};
  const Formula f3{sqrt(y_) / 4 != 1};
  const Formula f4{y_ > 25};

  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(f1 && f2 && f3 && f4, delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_NEAR(x, 4.0, delta_);  // f1
    EXPECT_TRUE(y >= std::abs(-x * x) - delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityLog) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == log(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::log(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityExp) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == exp(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::exp(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilitySin) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == sin(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::sin(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityCos) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == cos(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::cos(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityTan) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == tan(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::tan(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAsin) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == asin(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 0.5);
    EXPECT_NEAR(y, std::asin(0.5), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAcos) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == acos(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 0.5);
    EXPECT_NEAR(y, std::acos(0.5), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAtan) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == atan(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::atan(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAtan2) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        x_ == 4 && y_ == 3 && z_ == atan2(x_, y_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    const double z{solution.at(z_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_EQ(y, 3.0);
    EXPECT_NEAR(z, std::atan2(4.0, 3.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilitySinh) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == sinh(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 0.5);
    EXPECT_NEAR(y, std::sinh(0.5), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityCosh) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == cosh(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 0.5);
    EXPECT_NEAR(y, std::cosh(0.5), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityTanh) {
  if (solver_.available()) {
    const auto result =
        DrealSolver::CheckSatisfiability(x_ == 4 && y_ == tanh(x_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_NEAR(y, std::tanh(4.0), delta_);
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityMin) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        x_ == 4 && y_ == 3 && z_ == min(x_, y_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    const double z{solution.at(z_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_EQ(y, 3.0);
    EXPECT_EQ(z, std::min(4.0, 3.0));
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityMax) {
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        x_ == 4 && y_ == 3 && z_ == max(x_, y_), delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    const double y{solution.at(y_).mid()};
    const double z{solution.at(z_).mid()};
    EXPECT_EQ(x, 4.0);
    EXPECT_EQ(y, 3.0);
    EXPECT_EQ(z, std::max(4.0, 3.0));
  }
}

TEST_F(DrealSolverTest, CheckSatisfiabilityForall) {
  // To test `forall` formulas, encode the problem of minimizing x² in
  // exist-forall formula.
  //
  //     min x² s.t. x ∈ [-3, 3].
  // ->  ∃x. (-3 ≤ x) ∧ (x ≤ 3) ∧ [∀y. ((-3 ≤ y) ∧ (y ≤ 3)) → (x² ≤ y²)]
  // ->  ∃x. (-3 ≤ x) ∧ (x ≤ 3) ∧ [∀y. ¬((-3 ≤ y) ∧ (y ≤ 3)) ∨ (x² ≤ y²)]
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(
        (-3 <= x_) && (x_ <= 3) &&
            forall({y_}, !((-3 <= y_) && (y_ <= 3)) || (x_ * x_ <= y_ * y_)),
        delta_);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    EXPECT_NEAR(x, 0.0, delta_);
  }
}

TEST_F(DrealSolverTest, Minimize1) {
  // Minimize 2x² + 6x + 5 s.t. -4 ≤ x ≤ 0
  // The known minimum cost is -0.5.
  const Expression objective{2 * x_ * x_ + 6 * x_ + 5};
  const Formula constraint{-10 <= x_ && x_ <= 10};
  const double delta{0.01};
  const double known_minimum{0.5};

  if (solver_.available()) {
    const auto result = DrealSolver::Minimize(
        objective, constraint, delta, DrealSolver::LocalOptimization::kUse);
    ASSERT_TRUE(result);
    const DrealSolver::IntervalBox& solution{*result};
    const double x{solution.at(x_).mid()};
    EXPECT_TRUE(-10 <= x && x <= 10);
    EXPECT_LT(2 * x * x + 6 * x + 5, known_minimum + delta);
  }
}

TEST_F(DrealSolverTest, Minimize2) {
  // Minimize sin(3x) - 2cos(x) s.t. -3 ≤ x ≤ 3
  // The known minimum cost is -2.77877.
  const Expression objective{sin(3 * x_) - 2 * cos(x_)};
  const Formula constraint{-3 <= x_ && x_ <= 3};
  const double delta{0.001};
  const double known_minimum{-2.77877};
  if (solver_.available()) {
    const auto result = DrealSolver::Minimize(
        objective, constraint, delta, DrealSolver::LocalOptimization::kUse);
    ASSERT_TRUE(result);
    const double x{result->at(x_).mid()};
    EXPECT_TRUE(-3 <= x && x <= 3);
    EXPECT_LT(sin(3 * x) - 2 * cos(x), known_minimum + delta);
  }
}

TEST_F(DrealSolverTest, Minimize3) {
  // Minimize sin(3x) s.t. (-3 ≤ x ≤ 3) ∧ (x² - 16 = 0)
  // Note that the side constraints have no model.
  const Expression objective{sin(3 * x_)};
  const Formula constraint{-3 <= x_ && x_ <= 3 && (x_ * x_ - 16 == 0)};
  if (solver_.available()) {
    const auto result = DrealSolver::Minimize(
        objective, constraint, delta_, DrealSolver::LocalOptimization::kUse);
    EXPECT_FALSE(result);
  }
}

TEST_F(DrealSolverTest, UnsupportedFormulaIsnan) {
  if (solver_.available()) {
    EXPECT_THROW(DrealSolver::CheckSatisfiability(isnan(x_), delta_),
                 std::runtime_error);
  }
}

TEST_F(DrealSolverTest, UnsupportedFormulaCeil) {
  if (solver_.available()) {
    EXPECT_THROW(DrealSolver::CheckSatisfiability(ceil(x_) == 0, delta_),
                 std::runtime_error);
  }
}

TEST_F(DrealSolverTest, UnsupportedFormulaFloor) {
  if (solver_.available()) {
    EXPECT_THROW(DrealSolver::CheckSatisfiability(floor(x_) == 0, delta_),
                 std::runtime_error);
  }
}

TEST_F(DrealSolverTest, IfThenElse1) {
  const Formula f{x_ == 3 && y_ == 2 && z_ == if_then_else(x_ > y_, x_, y_)};
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f, delta_);
    ASSERT_TRUE(result);
    const double z{result->at(z_).mid()};
    EXPECT_EQ(z, 3);
  }
}

TEST_F(DrealSolverTest, IfThenElse2) {
  const Formula f{x_ == 2 && y_ == 3 && z_ == if_then_else(x_ > y_, x_, y_)};
  if (solver_.available()) {
    const auto result = DrealSolver::CheckSatisfiability(f, delta_);
    ASSERT_TRUE(result);
    const double z{result->at(z_).mid()};
    EXPECT_EQ(z, 3);
  }
}

TEST_F(DrealSolverTest, UnsupportedFormulaUninterpretedFunction) {
  if (solver_.available()) {
    EXPECT_THROW(
        DrealSolver::CheckSatisfiability(
            symbolic::uninterpreted_function("uf", {x_, y_}) == 0, delta_),
        std::runtime_error);
  }
}

TEST_F(DrealSolverTest, UnsupportedFormulaPositiveSemidefinite) {
  Eigen::Matrix<Expression, 2, 2> m;
  m << (x_ + y_), -1.0, -1.0, y_;
  if (solver_.available()) {
    EXPECT_THROW(
        DrealSolver::CheckSatisfiability(positive_semidefinite(m), delta_),
        std::runtime_error);
  }
}

TEST_F(DrealSolverTest, SolveLinearProgramming) {
  // Linear Cost + BoundingBox constraints + Linear constraint
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  prog_.AddConstraint(x0, 100, 200);
  prog_.AddConstraint(x1, 80, 170);
  prog_.AddConstraint(x1 >= -x0 + 200);
  prog_.AddCost(2 * x0 - 5 * x1);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double delta{0.001};
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    EXPECT_TRUE(100 - delta <= v0 && v0 <= 200 + delta);
    EXPECT_TRUE(80 - delta <= v1 && v1 <= 170 + delta);
    EXPECT_TRUE(v1 >= -v0 + 200 - delta);
    EXPECT_NEAR(2 * v0 - 5 * v1, 2 * 100 - 5 * 170 /* known minimum */,
                delta * 5.0);
  }
}

TEST_F(DrealSolverTest, SolveQuadraticProgramming) {
  // Linear Cost + BoundingBox constraints + Linear constraint
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  prog_.AddConstraint(x0, 0, 20);
  prog_.AddConstraint(x1 >= 0);
  prog_.AddConstraint(2 * x0 + x1 >= 2);
  prog_.AddConstraint(-x0 + 2 * x1 <= 6);
  prog_.AddCost(4 + 1.5 * x0 - 2 * x1 + 4 * x0 * x0 + 2 * x0 + x1 +
                5 * x1 * x1);
  const double delta{1e-5};
  prog_.SetSolverOption(DrealSolver::id(), "precision", delta);
  prog_.SetSolverOption(DrealSolver::id(), "use_local_optimization", 0);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    EXPECT_TRUE(0 - delta <= v0 && v0 <= 20 + delta);
    EXPECT_TRUE(0 - delta <= v1);
    EXPECT_TRUE(2 * v0 + v1 >= 2 - delta);
    EXPECT_NEAR(4 + 1.5 * v0 - 2 * v1 + 4 * v0 * v0 + 2 * v0 + v1 + 5 * v1 * v1,
                4 + 1.5 * 0.71875 - 2 * 0.5625 + 4 * 0.71875 * 0.71875 +
                    2 * 0.71875 + 0.5625 + 5 * 0.5625 * 0.5625,
                delta * 5.0);
  }
}

TEST_F(DrealSolverTest, SolveLinearEqualityConstraint) {
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  prog_.AddConstraint(x0, -5, 5);
  prog_.AddConstraint(x1, -5, 5);
  prog_.AddConstraint(2 * x0 + 3 * x1 == 2);
  prog_.AddConstraint(-3 * x0 + 4 * x1 <= 0);
  const double delta{1e-3};
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    EXPECT_NEAR(2 * v0 + 3 * v1, 2.0, delta);
    EXPECT_TRUE(-3 * v0 + 4 * v1 <= delta);
  }
}

TEST_F(DrealSolverTest, SolveQuadraticConstraint) {
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  prog_.AddConstraint(x0, 0, 5);
  prog_.AddConstraint(x1, 0, 5);
  prog_.AddConstraint(x0 <= x1 * x1);
  prog_.AddConstraint(x1 * x1 - 0.0001 <= x0);
  prog_.AddConstraint(x0 == 3.0);
  const double delta{1e-3};
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v1{result.GetSolution(x1)};
    EXPECT_NEAR(v1, 1.7320 /* sqrt(3.0) */, delta);
  }
}

TEST_F(DrealSolverTest, SolveLorentzConeConstraint) {
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  const Variable& x2{xvec_(2)};
  prog_.AddConstraint(x0, -5, 5);
  prog_.AddConstraint(x1, -5, 5);
  prog_.AddConstraint(x2, 0, 5);
  prog_.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(0 * x0 + 1, x0 - 1, x1 - 1));
  prog_.AddLorentzConeConstraint(Vector3<symbolic::Expression>(x2, x0, x1));
  prog_.AddCost(x2);
  const double delta{1e-5};
  prog_.SetSolverOption(DrealSolver::id(), "precision", delta);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v2{result.GetSolution(x2)};
    // We check if the found minimum (solution for x2) is close to the one from
    // SCS solver.
    EXPECT_NEAR(v2, /* Solution from SCS Solver */ 0.414212, delta * 5);
  }
}

TEST_F(DrealSolverTest, SolveRotatedLorentzConeConstraint) {
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  const Variable& x2{xvec_(2)};
  prog_.AddLinearCost(2 * x0 + 3 * x1 - 2 * x2);
  prog_.AddConstraint(x0, -1, 1);
  prog_.AddConstraint(x1, -1, 1);
  prog_.AddConstraint(x2, -1, 1);
  prog_.AddRotatedLorentzConeConstraint(
      Vector4<symbolic::Expression>(x0 + x1, x1 + x2, +x0, +x1));
  const double delta{1e-10};
  prog_.SetSolverOption(DrealSolver::id(), "precision", delta);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    const double v1{result.GetSolution(x1)};
    const double v2{result.GetSolution(x2)};
    // We check if the found minimum (solution for 2x0 + 3x1 - 2x2) is close to
    // the one from Gurobi solver.
    EXPECT_NEAR(2 * v0 + 3 * v1 - 2 * v2,
                /* Solution from Gurobi */
                2 * 0.0953487 + 3 * -0.0787482 - 2 * 1,
                1e-5);
  }
}

TEST_F(DrealSolverTest, SolveLinearComplementarityConstraint) {
  // The problem and the expected solution are copied from "bard1" test in
  // solvers/test/complementary_problem_test.cc.
  //
  // A problem from J.F. Bard, Convex two-level optimization,
  // Mathematical Programming 40(1), 15-27, 1988.
  // min (x-5)² + (2*y + 1)²
  // s.t 2*(y-1) - 1.5*x + l(0) - 0.5*l(1) + l(2) = 0
  //     0 <= l(0) ⊥ 3 * x - y - 3 >= 0
  //     0 <= l(1) ⊥ -x + 0.5*y + 4 >= 0
  //     0 <= l(2) ⊥ -x - y + 7 >= 0
  //     x >= 0, y >= 0
  const auto x = prog_.NewContinuousVariables<1>();
  const auto y = prog_.NewContinuousVariables<1>();
  const auto l = prog_.NewContinuousVariables<3>();
  prog_.AddCost(pow(x(0) - 5, 2) + pow(2 * y(0) + 1, 2));
  prog_.AddConstraint(x(0), -10, 10);
  prog_.AddConstraint(y(0), -10, 10);
  prog_.AddConstraint(l(0), -10, 10);
  prog_.AddConstraint(l(1), -10, 10);
  prog_.AddConstraint(l(2), -10, 10);
  prog_.AddLinearConstraint(
      2 * (y(0) - 1) - 1.5 * x(0) + l(0) - 0.5 * l(1) + l(2) == 0);
  Eigen::Matrix<double, 5, 5> M;
  // clang-format off
  M <<  3,  -1, 0, 0, 0,
       -1, 0.5, 0, 0, 0,
       -1,  -1, 0, 0, 0,
        0,   0, 0, 0, 0,
        0,   0, 0, 0, 0;
  // clang-format on
  Eigen::Matrix<double, 5, 1> q;
  q << -3, 4, 7, 0, 0;
  prog_.AddLinearComplementarityConstraint(M, q, {x, y, l});
  const double delta{1e-5};
  prog_.SetSolverOption(DrealSolver::id(), "precision", delta);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(x);
    const auto y_val = result.GetSolution(y);
    EXPECT_NEAR(x_val(0), 1, 1E-6);
    EXPECT_NEAR(y_val(0), 0, 1E-6);
  }
}

TEST_F(DrealSolverTest, SolveNonLinearConstraint) {
  const Variable& x0{xvec_(0)};
  prog_.AddConstraint(x0, -3.141592, 3.141592);
  prog_.AddConstraint(sin(x0) + cos(x0), 0.4, 0.41);
  const double delta{1e-5};
  prog_.SetSolverOption(DrealSolver::id(), "precision", delta);
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v0{result.GetSolution(x0)};
    EXPECT_TRUE(-3.141592 - delta <= v0 && v0 <= 3.141592 + delta);
    EXPECT_TRUE(0.4 - delta <= sin(v0) + cos(v0));
    EXPECT_TRUE(sin(v0) + cos(v0) <= 0.41 + delta);
    // Add more constraints to make the problem infeasible.
    prog_.AddConstraint(cos(x0) * sin(x0), 0.9, 0.91);
    solver_.Solve(prog_, {}, {}, &result);
    ASSERT_FALSE(result.is_success());
    EXPECT_EQ(result.get_solution_result(),
              SolutionResult::kInfeasibleConstraints);
  }
}

TEST_F(DrealSolverTest, SolvePositiveSemidefiniteConstraint) {
  // No support yet. Checks DrealSolver throws std::logic_error.
  const auto X = prog_.NewSymmetricContinuousVariables<4>("X");
  prog_.AddPositiveSemidefiniteConstraint(X);
  EXPECT_THROW(solver_.Solve(prog_, {}, {}), logic_error);
}

TEST_F(DrealSolverTest, SolveLinearMatrixInequalityConstraint) {
  // No support yet. Checks DrealSolver throws std::logic_error.
  prog_.AddLinearMatrixInequalityConstraint(
      {Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Ones(),
       2 * Eigen::Matrix2d::Ones()},
      xvec_.head<2>());
  if (solver_.available()) {
    EXPECT_THROW(solver_.Solve(prog_, {}, {}), logic_error);
  }
}

TEST_F(DrealSolverTest, SolveMultipleCostFunctions) {
  const Variable& x{xvec_(0)};
  prog_.AddConstraint(x, -10, 10);
  prog_.AddCost(-2 * x + 1);  // -2x + 1
  prog_.AddCost(x * x);       // x²
  if (solver_.available()) {
    // Cost function = x² - 2x + 1 = (x-1)²
    auto result = solver_.Solve(prog_, {}, {});
    ASSERT_TRUE(result.is_success());
    const double v{result.GetSolution(x)};
    EXPECT_NEAR(v, 1, 0.005);
  }
}

TEST_F(DrealSolverTest, SolveGenericConstraint) {
  const Variable& x0{xvec_(0)};
  const Variable& x1{xvec_(1)};
  const Variable& x2{xvec_(2)};
  prog_.AddConstraint(x0, -10, 10);
  prog_.AddConstraint(x1, -10, 10);
  prog_.AddConstraint(x2, -10, 10);
  // -1 <= x0 * x1 + x2 / x0 * 2 <= 2
  // -2 <= x1 * x2 - x0 <= 1
  const shared_ptr<Constraint> generic_trivial_constraint1 =
      make_shared<test::GenericTrivialConstraint1>();
  prog_.AddConstraint(Binding<Constraint>(
      generic_trivial_constraint1, VectorDecisionVariable<3>(x0, x1, x2)));
  if (solver_.available()) {
    auto result = solver_.Solve(prog_, {}, {});
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

TEST_F(DrealSolverTest, SolveGenericCost) {
  // No support yet. Checks DrealSolver throws std::logic_error.
  const shared_ptr<Cost> generic_trivial_cost1 =
      make_shared<test::GenericTrivialCost1>();
  prog_.AddCost(
      Binding<Cost>(generic_trivial_cost1,
                    VectorDecisionVariable<3>(xvec_(0), xvec_(1), xvec_(2))));
  EXPECT_THROW(solver_.Solve(prog_, {}, {}), logic_error);
}

}  // namespace

}  // namespace solvers
}  // namespace drake
