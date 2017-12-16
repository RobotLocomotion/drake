#include "drake/solvers/dreal_solver.h"

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace {

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::Variables;

class DrealSolverTest : public ::testing::Test {
 protected:
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
};

// 0.0 > 1.0 is trivially UNSAT.
TEST_F(DrealSolverTest, CheckSatisfiabilityTrivialUnsat) {
  const auto result = DrealSolver::CheckSatisfiability(
      Expression{0.0} > Expression{1.0}, delta_);
  ASSERT_FALSE(result);
}

// 1.0 > 0.0 is trivially SAT.
TEST_F(DrealSolverTest, CheckSatisfiabilityTrivialSat) {
  const auto result = DrealSolver::CheckSatisfiability(
      Expression{1.0} > Expression{0.0}, delta_);
  ASSERT_TRUE(result);
  // The result is an empty box.
  EXPECT_EQ(result->size(), 0.0);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityConjunction) {
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

TEST_F(DrealSolverTest, CheckSatisfiabilityDisjunction) {
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

TEST_F(DrealSolverTest, CheckSatisfiabilityLinearReal) {
  const Formula f1{0 <= x_ && x_ <= 5};
  const Formula f2{0 <= y_ && y_ <= 5};
  const Formula f3{2 * x_ + 3 * y_ == 5 && -3 * x_ + 4 * y_ == 6};

  const auto result = DrealSolver::CheckSatisfiability(f1 && f2 && f3, delta_);
  ASSERT_TRUE(result);

  const DrealSolver::IntervalBox& solution{*result};
  const double expected_x{2.0 / 17.0};
  const double expected_y{27.0 / 17.0};
  EXPECT_NEAR(solution.at(x_).mid(), expected_x, delta_);
  EXPECT_NEAR(solution.at(y_).mid(), expected_y, delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityLinearInteger) {
  const Formula f1{0 <= i_ && i_ <= 5};
  const Formula f2{0 <= j_ && j_ <= 5};
  const Formula f3{2 * i_ + 3 * j_ == 5 && -3 * i_ + 4 * j_ == 6};
  const auto result = DrealSolver::CheckSatisfiability(f1 && f2 && f3, delta_);
  // Note that this has the same constraint as the previous,
  // CheckSatisfiabilityLinearReal test. However, the domain constraint, i,j ∈
  // Z, makes the problem unsatisfiable.
  EXPECT_FALSE(result);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityBinaryVariables) {
  const Formula f{2 * binary1_ + 3 * binary2_ == 0};
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

// Tests CheckSatisfiability (UNSAT case).
TEST_F(DrealSolverTest, CheckSatisfiabilityUnsat) {
  // Find a model satisfying the following constraints:
  //     2x² + 6x + 5 < 0
  //     -10 ≤ x ≤ 10
  const Formula f1{2 * x_ * x_ + 6 * x_ + 5 < 0};
  const Formula f2{-10 <= x_ && x_ <= 10};

  const auto result = DrealSolver::CheckSatisfiability(f1 && f2, delta_);
  EXPECT_FALSE(result);
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

  const auto result =
      DrealSolver::CheckSatisfiability(f1 && f2 && f3 && f4, delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_NEAR(x, 4.0, delta_);  // f1
  EXPECT_TRUE(y >= std::abs(-x * x) - delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityLog) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == log(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::log(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityExp) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == exp(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::exp(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilitySin) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == sin(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::sin(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityCos) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == cos(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::cos(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityTan) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == tan(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::tan(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAsin) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == asin(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 0.5);
  EXPECT_NEAR(y, std::asin(0.5), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAcos) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == acos(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 0.5);
  EXPECT_NEAR(y, std::acos(0.5), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAtan) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == atan(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::atan(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityAtan2) {
  auto result = DrealSolver::CheckSatisfiability(
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

TEST_F(DrealSolverTest, CheckSatisfiabilitySinh) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == sinh(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 0.5);
  EXPECT_NEAR(y, std::sinh(0.5), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityCosh) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 0.5 && y_ == cosh(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 0.5);
  EXPECT_NEAR(y, std::cosh(0.5), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityTanh) {
  auto result =
      DrealSolver::CheckSatisfiability(x_ == 4 && y_ == tanh(x_), delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  const double y{solution.at(y_).mid()};
  EXPECT_EQ(x, 4.0);
  EXPECT_NEAR(y, std::tanh(4.0), delta_);
}

TEST_F(DrealSolverTest, CheckSatisfiabilityMin) {
  auto result = DrealSolver::CheckSatisfiability(
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

TEST_F(DrealSolverTest, CheckSatisfiabilityMax) {
  auto result = DrealSolver::CheckSatisfiability(
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

TEST_F(DrealSolverTest, CheckSatisfiabilityForall) {
  // To test `forall` formulas, encode the problem of minimizing x² in
  // exist-forall formula.
  //
  //     min x² s.t. x ∈ [-3, 3].
  // ->  ∃x. (-3 ≤ x) ∧ (x ≤ 3) ∧ [∀y. ((-3 ≤ y) ∧ (y ≤ 3)) → (x² ≤ y²)]
  // ->  ∃x. (-3 ≤ x) ∧ (x ≤ 3) ∧ [∀y. ¬((-3 ≤ y) ∧ (y ≤ 3)) ∨ (x² ≤ y²)]
  auto result = DrealSolver::CheckSatisfiability(
      (-3 <= x_) && (x_ <= 3) &&
          forall({y_}, !((-3 <= y_) && (y_ <= 3)) || (x_ * x_ <= y_ * y_)),
      delta_);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  EXPECT_NEAR(x, 0.0, delta_);
}

TEST_F(DrealSolverTest, Minimize1) {
  // Minimize 2x² + 6x + 5 s.t. -4 ≤ x ≤ 0
  // The known minimum cost is -0.5.
  const Expression objective{2 * x_ * x_ + 6 * x_ + 5};
  const Formula constraint{-10 <= x_ && x_ <= 10};
  const double delta{0.01};
  const double known_minimum{0.5};

  const auto result = DrealSolver::Minimize(objective, constraint, delta);
  ASSERT_TRUE(result);
  const DrealSolver::IntervalBox& solution{*result};
  const double x{solution.at(x_).mid()};
  EXPECT_TRUE(-10 <= x && x <= 10);
  EXPECT_LT(2 * x * x + 6 * x + 5, known_minimum + delta);
}

TEST_F(DrealSolverTest, Minimize2) {
  // Minimize sin(3x) - 2cos(x) s.t. -3 ≤ x ≤ 3
  // The known minimum cost is -2.77877.
  const Expression objective{sin(3 * x_) - 2 * cos(x_)};
  const Formula constraint{-3 <= x_ && x_ <= 3};
  const double delta{0.001};
  const double known_minimum{-2.77877};

  const auto result = DrealSolver::Minimize(objective, constraint, delta);
  ASSERT_TRUE(result);
  const double x{result->at(x_).mid()};
  EXPECT_TRUE(-3 <= x && x <= 3);
  EXPECT_LT(sin(3 * x) - 2 * cos(x), known_minimum + delta);
}

TEST_F(DrealSolverTest, Minimize3) {
  // Minimize sin(3x) s.t. (-3 ≤ x ≤ 3) ∧ (x² - 16 = 0)
  // Note that the side constraints have no model.
  const Expression objective{sin(3 * x_)};
  const Formula constraint{-3 <= x_ && x_ <= 3 && (x_ * x_ - 16 == 0)};

  const auto result = DrealSolver::Minimize(objective, constraint, delta_);
  EXPECT_FALSE(result);
}

TEST_F(DrealSolverTest, UnsupportedFormulaIsnan) {
  EXPECT_THROW(DrealSolver::CheckSatisfiability(isnan(x_), delta_),
               std::runtime_error);
}

TEST_F(DrealSolverTest, UnsupportedFormulaCeil) {
  EXPECT_THROW(DrealSolver::CheckSatisfiability(ceil(x_) == 0, delta_),
               std::runtime_error);
}

TEST_F(DrealSolverTest, UnsupportedFormulaFloor) {
  EXPECT_THROW(DrealSolver::CheckSatisfiability(floor(x_) == 0, delta_),
               std::runtime_error);
}

TEST_F(DrealSolverTest, UnsupportedFormulaIfThenElse) {
  EXPECT_THROW(DrealSolver::CheckSatisfiability(
                   if_then_else(x_ > y_, x_, y_) == 0, delta_),
               std::runtime_error);
}

TEST_F(DrealSolverTest, UnsupportedFormulaUninterpretedFunction) {
  EXPECT_THROW(
      DrealSolver::CheckSatisfiability(
          uninterpreted_function("uf", Variables{x_, y_}) == 0, delta_),
      std::runtime_error);
}

TEST_F(DrealSolverTest, UnsupportedFormulaPositiveSemidefinite) {
  Eigen::Matrix<Expression, 2, 2> m;
  m << (x_ + y_), -1.0, -1.0, y_;
  EXPECT_THROW(
      DrealSolver::CheckSatisfiability(positive_semidefinite(m), delta_),
      std::runtime_error);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
