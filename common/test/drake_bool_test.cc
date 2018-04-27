#include "drake/common/drake_bool.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {

using std::move;

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Variable;
using symbolic::test::ExprEqual;
using symbolic::test::FormulaEqual;

// --------------------------------------------------------------
// Test constructors and assignment operators using T = double case.
// --------------------------------------------------------------
class BoolTestDouble : public ::testing::Test {
 protected:
  Bool<double> b_true_{3.0 < 4.0};
  Bool<double> b_false_{3.0 > 4.0};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
};

TEST_F(BoolTestDouble, Constructor) {
  EXPECT_TRUE(v_true_);
  EXPECT_FALSE(v_false_);
}

TEST_F(BoolTestDouble, CopyConstructor) {
  const Bool<double> b_true_copy{b_true_};
  const Bool<double> b_false_copy{b_false_};
  EXPECT_TRUE(ExtractBoolOrThrow(b_true_copy));
  EXPECT_FALSE(ExtractBoolOrThrow(b_false_copy));
}

TEST_F(BoolTestDouble, MoveConstructor) {
  const Bool<double> b_true_moved{move(b_true_)};
  const Bool<double> b_false_moved{move(b_false_)};
  EXPECT_TRUE(ExtractBoolOrThrow(b_true_moved));
  EXPECT_FALSE(ExtractBoolOrThrow(b_false_moved));
}

TEST_F(BoolTestDouble, CopyAssign) {
  Bool<double> b1{b_false_};
  b1 = b_true_;
  EXPECT_TRUE(ExtractBoolOrThrow(b1));

  Bool<double> b2{b_true_};
  b2 = b_false_;
  EXPECT_FALSE(ExtractBoolOrThrow(b2));
}

TEST_F(BoolTestDouble, MoveAssign) {
  Bool<double> b1{b_false_};
  b1 = move(b_true_);
  EXPECT_TRUE(ExtractBoolOrThrow(b1));

  Bool<double> b2{b_true_};
  b2 = move(b_false_);
  EXPECT_FALSE(ExtractBoolOrThrow(b2));
}

TEST_F(BoolTestDouble, TypeCheck) {
  static_assert(std::is_same<Bool<double>::value_type, bool>::value,
                "Bool<double>::value_type should be bool");
}

TEST_F(BoolTestDouble, Value) {
  EXPECT_TRUE(b_true_.value());
  EXPECT_FALSE(b_false_.value());
}

TEST_F(BoolTestDouble, IfThenElse) {
  const double v_true{if_then_else(b_true_, 1.0, 0.0)};
  const double v_false{if_then_else(b_false_, 1.0, 0.0)};
  EXPECT_EQ(v_true, 1);
  EXPECT_EQ(v_false, 0);
}

TEST_F(BoolTestDouble, Cond) {
  const double v_true{cond(b_true_, 1.0, 0.0)};
  const double v_false{cond(b_false_, 1.0, 0.0)};
  EXPECT_EQ(v_true, 1);
  EXPECT_EQ(v_false, 0);
}

TEST_F(BoolTestDouble, LogicalOperators) {
  EXPECT_TRUE((b_true_ && b_true_).value());
  EXPECT_FALSE((b_false_ && b_true_).value());
  EXPECT_FALSE((b_true_ && b_false_).value());
  EXPECT_FALSE((b_false_ && b_false_).value());
  EXPECT_FALSE((b_true_ && false).value());
  EXPECT_FALSE((false && b_true_).value());

  EXPECT_TRUE((b_true_ || b_true_).value());
  EXPECT_TRUE((b_false_ || b_true_).value());
  EXPECT_TRUE((b_true_ || b_false_).value());
  EXPECT_FALSE((b_false_ || b_false_).value());
  EXPECT_TRUE((b_false_ || true).value());
  EXPECT_TRUE((true || b_false_).value());

  EXPECT_FALSE((!b_true_).value());
  EXPECT_TRUE((!b_false_).value());
}

// -------------------
// Case T = AutoDiffXd
// -------------------
class BoolTestAutoDiffXd : public ::testing::Test {
 protected:
  const AutoDiffXd x_{3., Eigen::Vector2d{1., 0.}};
  const AutoDiffXd y_{4., Eigen::Vector2d{0., 1.}};
  Bool<AutoDiffXd> b_true_{x_ < y_};
  Bool<AutoDiffXd> b_false_{x_ > y_};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
};

TEST_F(BoolTestAutoDiffXd, TypeCheck) {
  static_assert(std::is_same<Bool<AutoDiffXd>::value_type, bool>::value,
                "Bool<AutoDiffXd>::value_type should be bool");
}

TEST_F(BoolTestAutoDiffXd, ExtractBoolOrThrow) {
  EXPECT_TRUE(v_true_);
  EXPECT_FALSE(v_false_);
}

TEST_F(BoolTestAutoDiffXd, Value) {
  EXPECT_TRUE(b_true_.value());
  EXPECT_FALSE(b_false_.value());
}

TEST_F(BoolTestAutoDiffXd, IfThenElse) {
  const AutoDiffXd v_true{if_then_else(b_true_, x_, y_)};
  const AutoDiffXd v_false{if_then_else(b_false_, x_, y_)};
  EXPECT_EQ(v_true, x_);
  EXPECT_EQ(v_false, y_);
}

TEST_F(BoolTestAutoDiffXd, Cond) {
  const AutoDiffXd v_true{cond(b_true_, x_, y_)};
  const AutoDiffXd v_false{cond(b_false_, x_, y_)};
  EXPECT_EQ(v_true, x_);
  EXPECT_EQ(v_false, y_);
}

TEST_F(BoolTestAutoDiffXd, LogicalOperators) {
  EXPECT_TRUE((b_true_ && b_true_).value());
  EXPECT_FALSE((b_false_ && b_true_).value());
  EXPECT_FALSE((b_true_ && b_false_).value());
  EXPECT_FALSE((b_false_ && b_false_).value());
  EXPECT_FALSE((b_true_ && (x_ > y_)).value());
  EXPECT_FALSE(((x_ > y_) && b_true_).value());

  EXPECT_TRUE((b_true_ || b_true_).value());
  EXPECT_TRUE((b_false_ || b_true_).value());
  EXPECT_TRUE((b_true_ || b_false_).value());
  EXPECT_FALSE((b_false_ || b_false_).value());
  EXPECT_TRUE((b_false_ || (x_ < y_)).value());
  EXPECT_TRUE(((x_ < y_) || b_false_).value());

  EXPECT_FALSE((!b_true_).value());
  EXPECT_TRUE((!b_false_).value());
}

// -----------------------------
// Case T = symbolic::Expression
// -----------------------------
class BoolTestSymbolic : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  Bool<Expression> b_true_{Expression{3.0} < Expression{4.0}};
  Bool<Expression> b_false_{Expression{3.0} > Expression{4.0}};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
};

TEST_F(BoolTestSymbolic, TypeCheck) {
  static_assert(
      std::is_same<Bool<Expression>::value_type, Formula>::value,
      "Bool<symbolic::Expression>::value_type should be symbolic::Formula");
}

TEST_F(BoolTestSymbolic, ConstructFromBool) {
  // Checks `Bool(bool b)`.
  const Bool<Expression> b_true{true};
  const Bool<Expression> b_false{false};
  EXPECT_PRED2(FormulaEqual, b_true.value(), Formula::True());
  EXPECT_PRED2(FormulaEqual, b_false.value(), Formula::False());
}

TEST_F(BoolTestSymbolic, ExtractBoolOrThrowNoException) {
  EXPECT_TRUE(v_true_);
  EXPECT_FALSE(v_false_);
}

TEST_F(BoolTestSymbolic, ExtractBoolOrThrowException) {
  // A symbolic formula `x < y` cannot be converted to a Boolean value without
  // the assignments on `x` and `y`.
  const Bool<Expression> b{x_ < y_};
  EXPECT_THROW(ExtractBoolOrThrow(b), std::exception);
}

TEST_F(BoolTestSymbolic, Value) {
  EXPECT_PRED2(FormulaEqual, b_true_.value(), Formula::True());
  EXPECT_PRED2(FormulaEqual, b_false_.value(), Formula::False());
  EXPECT_PRED2(FormulaEqual, Bool<Expression>{x_ < y_}.value(), x_ < y_);
}

TEST_F(BoolTestSymbolic, IfThenElse) {
  const Expression e{if_then_else(Bool<Expression>{x_ > y_}, +x_, -y_)};

  Environment env;
  env[x_] = 4.0;
  env[y_] = 3.0;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), env[x_]);

  env[x_] = 3.0;
  env[y_] = 4.0;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), -env[y_]);
}

TEST_F(BoolTestSymbolic, Cond) {
  // clang-format off
  const Expression e{cond(x_ > 3.0, 3.0,
                          x_ > 2.0, 2.0,
                          x_ > 1.0, 1.0,
                          0.0)};
  // clang-format on
  Environment env;
  env[x_] = 3.5;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), 3.0);

  env[x_] = 2.5;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), 2.0);

  env[x_] = 1.5;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), 1.0);

  env[x_] = 0.5;
  EXPECT_PRED2(ExprEqual, e.Evaluate(env), 0.0);
}

TEST_F(BoolTestSymbolic, LogicalOperators) {
  const Formula f1{x_ > 0};
  const Formula f2{y_ > 0};
  const Bool<Expression> b1{f1};
  const Bool<Expression> b2{f2};

  EXPECT_PRED2(FormulaEqual, (b1 && b2).value(), f1 && f2);
  EXPECT_PRED2(FormulaEqual, (b1 && f2).value(), f1 && f2);
  EXPECT_PRED2(FormulaEqual, (f1 && b2).value(), f1 && f2);

  EXPECT_PRED2(FormulaEqual, (b1 || b2).value(), f1 || f2);
  EXPECT_PRED2(FormulaEqual, (b1 || f2).value(), f1 || f2);
  EXPECT_PRED2(FormulaEqual, (f1 || b2).value(), f1 || f2);

  EXPECT_PRED2(FormulaEqual, (!b1).value(), !f1);
}

}  // namespace drake
