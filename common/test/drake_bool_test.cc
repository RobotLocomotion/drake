#include "drake/common/drake_bool.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {

using std::move;

using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::test::FormulaEqual;

// --------------------------------------------------------------
// Test constructors and assignment operators in T = double case.
// --------------------------------------------------------------
GTEST_TEST(BoolTest, Constructor) {
  const bool_t<double> b{3.0 < 4.0};
  EXPECT_TRUE(ExtractBoolOrThrow(b));
}

GTEST_TEST(BoolTest, CopyConstructor) {
  const bool_t<double> b1{3.0 < 4.0};
  const bool_t<double> b2{b1};
  EXPECT_EQ(ExtractBoolOrThrow(b1), ExtractBoolOrThrow(b2));
}

GTEST_TEST(BoolTest, MoveConstructor) {
  bool_t<double> b1{3.0 < 4.0};
  const bool v1{ExtractBoolOrThrow(b1)};

  const bool_t<double> b2{move(b1)};
  const bool v2{ExtractBoolOrThrow(b2)};

  EXPECT_EQ(v1, v2);
}

GTEST_TEST(BoolTest, CopyAssign) {
  const bool_t<double> b1{3.0 < 4.0};
  bool_t<double> b2{3.0 > 4.0};
  b2 = b1;
  EXPECT_EQ(ExtractBoolOrThrow(b1), ExtractBoolOrThrow(b2));
}

GTEST_TEST(BoolTest, MoveAssign) {
  bool_t<double> b1{3.0 < 4.0};
  const bool v1{ExtractBoolOrThrow(b1)};

  bool_t<double> b2{3.0 > 4.0};
  b2 = move(b1);
  const bool v2{ExtractBoolOrThrow(b2)};

  EXPECT_EQ(v1, v2);
}

// ---------------
// Case T = double
// ---------------
GTEST_TEST(BoolTest, DoubleTypeCheck) {
  static_assert(std::is_same<bool_t<double>::formula_t, bool>::value,
                "bool_t<double>::formula should be bool");
}

GTEST_TEST(BoolTest, DoubleValue) {
  const bool_t<double> b{3.0 < 4.0};
  EXPECT_EQ(b.value(), 3.0 < 4.0);
}

// -------------------
// Case T = AutoDiffXd
// -------------------
GTEST_TEST(BoolTest, AutoDiffXdTypeCheck) {
  static_assert(std::is_same<bool_t<AutoDiffXd>::formula_t, bool>::value,
                "bool_t<AutoDiffXd>::formula should be bool");
}

GTEST_TEST(BoolTest, AutoDiffXdExtractBoolOrThrow) {
  const AutoDiffXd x(3., Eigen::Vector2d{1., 0.});
  const AutoDiffXd y(4., Eigen::Vector2d{0., 1.});
  const bool_t<AutoDiffXd> b{x < y};
  const bool v{ExtractBoolOrThrow(b)};
  EXPECT_TRUE(v);
}

GTEST_TEST(BoolTest, AutoDiffXdValue) {
  const AutoDiffXd x(3., Eigen::Vector2d{1., 0.});
  const AutoDiffXd y(4., Eigen::Vector2d{0., 1.});
  const bool_t<AutoDiffXd> b{x < y};
  EXPECT_EQ(b.value(), x < y);
}

// -----------------------------
// Case T = symbolic::Expression
// -----------------------------
GTEST_TEST(BoolTest, SymbolicTypeCheck) {
  static_assert(
      std::is_same<bool_t<Expression>::formula_t, Formula>::value,
      "bool_t<symbolic::Expression>::formula should be symbolic::Formula");
}

GTEST_TEST(BoolTest, SymbolicConstructFromBool) {
  const bool_t<Expression> b_false{false};
  const bool_t<Expression> b_true{true};
  EXPECT_PRED2(FormulaEqual, b_false.value(), Formula::False());
  EXPECT_PRED2(FormulaEqual, b_true.value(), Formula::True());
}

GTEST_TEST(BoolTest, SymbolicExtractBoolOrThrow1) {
  const bool_t<Expression> b{Expression{3.0} < Expression{4.0}};
  const bool v{ExtractBoolOrThrow(b)};
  EXPECT_TRUE(v);
}

GTEST_TEST(BoolTest, SymbolicExtractBoolOrThrow2) {
  // A symbolic formula `x < y` cannot be converted to a Boolean value without
  // the assignments on `x` and `y`.
  const Variable x{"x"};
  const Variable y{"y"};
  const bool_t<Expression> b{x < y};
  EXPECT_THROW(ExtractBoolOrThrow(b), std::exception);
}

GTEST_TEST(BoolTest, SymbolicValue) {
  const Variable x{"x"};
  const Variable y{"y"};
  const bool_t<Expression> b{x < y};
  EXPECT_PRED2(FormulaEqual, b.value(), x < y);
}

}  // namespace drake
