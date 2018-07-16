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
  void SetUp() override {
    // clang-format off
    m_ << 1, 2,
          3, 4;
    // clang-format on
  }

  Bool<double> b_true_{3.0 < 4.0};
  Bool<double> b_false_{3.0 > 4.0};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
  MatrixX<double> m_{2, 2};
  MatrixX<double> zero_m_{0, 0};
};

TEST_F(BoolTestDouble, Constructor) {
  EXPECT_TRUE(v_true_);
  EXPECT_FALSE(v_false_);
}

TEST_F(BoolTestDouble, TrueFalse) {
  EXPECT_TRUE(Bool<double>::True().value());
  EXPECT_FALSE(Bool<double>::False().value());
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

TEST_F(BoolTestDouble, AllOf) {
  EXPECT_TRUE(all_of(m_, [](const double v) { return v >= 0.0; }).value());
  EXPECT_FALSE(all_of(m_, [](const double v) { return v >= 2.0; }).value());

  // Vacuously true.
  EXPECT_TRUE(all_of(zero_m_, [](const double v) { return v >= 0.0; }).value());
}

TEST_F(BoolTestDouble, AnyOf) {
  EXPECT_TRUE(any_of(m_, [](const double v) { return v >= 4.0; }).value());
  EXPECT_FALSE(any_of(m_, [](const double v) { return v >= 5.0; }).value());

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const double v) { return v >= 0.0; }).value());
}

TEST_F(BoolTestDouble, NoneOf) {
  EXPECT_TRUE(none_of(m_, [](const double v) { return v >= 5.0; }).value());
  EXPECT_FALSE(none_of(m_, [](const double v) { return v >= 4.0; }).value());

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const double v) { return v >= 0.0; }).value());
}

// -------------------
// Case T = AutoDiffXd
// -------------------
class BoolTestAutoDiffXd : public ::testing::Test {
 protected:
  void SetUp() override {
    m_(0, 0) = AutoDiffXd{1., Eigen::Vector2d{1., 0.}};
    m_(0, 1) = AutoDiffXd{0., Eigen::Vector2d{1., 0.}};
    m_(1, 0) = AutoDiffXd{0., Eigen::Vector2d{0., 1.}};
    m_(1, 1) = AutoDiffXd{1., Eigen::Vector2d{0., 1.}};
  }

  const AutoDiffXd x_{3., Eigen::Vector2d{1., 0.}};
  const AutoDiffXd y_{4., Eigen::Vector2d{0., 1.}};
  Bool<AutoDiffXd> b_true_{x_ < y_};
  Bool<AutoDiffXd> b_false_{x_ > y_};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
  MatrixX<AutoDiffXd> m_{2, 2};
  MatrixX<AutoDiffXd> zero_m_{0, 0};
};

TEST_F(BoolTestAutoDiffXd, TypeCheck) {
  static_assert(std::is_same<Bool<AutoDiffXd>::value_type, bool>::value,
                "Bool<AutoDiffXd>::value_type should be bool");
}

TEST_F(BoolTestAutoDiffXd, TrueFalse) {
  EXPECT_TRUE(Bool<AutoDiffXd>::True().value());
  EXPECT_FALSE(Bool<AutoDiffXd>::False().value());
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

TEST_F(BoolTestAutoDiffXd, AllOf) {
  EXPECT_TRUE(all_of(m_,
                     [](const AutoDiffXd& v) {
                       return v.derivatives()[0] == 1.0 ||
                              v.derivatives()[1] == 1.0;
                     })
                  .value());
  EXPECT_FALSE(
      all_of(m_, [](const AutoDiffXd& v) { return v >= 1.0; }).value());

  // Vacuously true.
  EXPECT_TRUE(
      all_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }).value());
}

TEST_F(BoolTestAutoDiffXd, AnyOf) {
  EXPECT_TRUE(any_of(m_,
                     [](const AutoDiffXd& v) {
                       return v.derivatives()[0] == 1.0 &&
                              v.derivatives()[1] == 0.0;
                     })
                  .value());
  EXPECT_FALSE(any_of(m_,
                      [](const AutoDiffXd& v) {
                        return v.derivatives()[0] == 1.0 &&
                               v.derivatives()[1] == 1.0;
                      })
                   .value());

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }).value());
}

TEST_F(BoolTestAutoDiffXd, NoneOf) {
  EXPECT_TRUE(none_of(m_,
                      [](const AutoDiffXd& v) {
                        return v.derivatives()[0] == 1.0 &&
                               v.derivatives()[1] == 1.0;
                      })
                  .value());
  EXPECT_FALSE(none_of(m_,
                       [](const AutoDiffXd& v) {
                         return v.derivatives()[0] == 1.0 &&
                                v.derivatives()[1] == 0.0;
                       })
                   .value());

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const AutoDiffXd& v) { return v >= 1.0; }).value());
}

// -----------------------------
// Case T = symbolic::Expression
// -----------------------------
class BoolTestSymbolic : public ::testing::Test {
 protected:
  void SetUp() override {
    // clang-format off
    m_ <<  x_, y_,
           z_, w_;
    // clang-format on
  }

  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
  Bool<Expression> b_true_{Expression{3.0} < Expression{4.0}};
  Bool<Expression> b_false_{Expression{3.0} > Expression{4.0}};
  const bool v_true_{ExtractBoolOrThrow(b_true_)};
  const bool v_false_{ExtractBoolOrThrow(b_false_)};
  MatrixX<Expression> m_{2, 2};
  MatrixX<Expression> zero_m_{0, 0};
};

TEST_F(BoolTestSymbolic, TypeCheck) {
  static_assert(
      std::is_same<Bool<Expression>::value_type, Formula>::value,
      "Bool<symbolic::Expression>::value_type should be symbolic::Formula");
}

TEST_F(BoolTestSymbolic, TrueFalse) {
  EXPECT_TRUE(is_true(Bool<Expression>::True().value()));
  EXPECT_TRUE(is_false(Bool<Expression>::False().value()));
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

// Check if ExtractBoolOrThrow works consistently across the relational
// operators (here, we only check == and <=).
// See https://github.com/RobotLocomotion/drake/issues/9117.
TEST_F(BoolTestSymbolic, ExtractBoolOrThrowConsistency) {
  // These tests throw an exception because it is not possible to determine how
  // x relates to 2 (there is no numerical value associated with x).
  Bool<Expression> test_equal{x_ == 2};
  Bool<Expression> test_less_than_equal{x_ <= 2};
  EXPECT_THROW(ExtractBoolOrThrow(test_equal), std::runtime_error);
  EXPECT_THROW(ExtractBoolOrThrow(test_less_than_equal), std::runtime_error);

  // These tests do not throw an exception because the symbolic algorithms are
  // able to definitely resolve the relationship between these expressions.
  test_equal = (2 * x_ + 3 == 3 + 2 * x_);
  test_less_than_equal = (2 * x_ + 3 <= 3 + 2 * x_);
  EXPECT_TRUE(ExtractBoolOrThrow(test_equal));
  EXPECT_TRUE(ExtractBoolOrThrow(test_less_than_equal));

  // These tests throw an exception because the symbolic algorithms do not
  // currently definitely resolve the relationship between these expressions.
  //
  // TODO(Mitiguy): Update this test as more symbolic algorithms are employed to
  // resolve the comparison between these expressions.
  test_equal = 2 * (x_ + 3) == 6 + 2 * x_;
  test_less_than_equal = 2 * (x_ + 3) <= 6 + 2 * x_;
  EXPECT_THROW(ExtractBoolOrThrow(test_equal), std::runtime_error);
  EXPECT_THROW(ExtractBoolOrThrow(test_less_than_equal), std::runtime_error);
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

TEST_F(BoolTestSymbolic, AllOf) {
  EXPECT_PRED2(
      FormulaEqual,
      all_of(m_, [](const Expression& v) { return !isnan(v); }).value(),
      !isnan(x_) && !isnan(y_) && !isnan(z_) && !isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(
      all_of(zero_m_, [](const Expression& v) { return v >= 0.0; }).value());
}

TEST_F(BoolTestSymbolic, AnyOf) {
  EXPECT_PRED2(FormulaEqual,
               any_of(m_, [](const Expression& v) { return isnan(v); }).value(),
               isnan(x_) || isnan(y_) || isnan(z_) || isnan(w_));

  // Vacuously false.
  EXPECT_FALSE(
      any_of(zero_m_, [](const Expression& v) { return v >= 0.0; }).value());
}

TEST_F(BoolTestSymbolic, NoneOf) {
  EXPECT_PRED2(
      FormulaEqual,
      none_of(m_, [](const Expression& v) { return isnan(v); }).value(),
      !isnan(x_) && !isnan(y_) && !isnan(z_) && !isnan(w_));

  // Vacuously true.
  EXPECT_TRUE(
      none_of(zero_m_, [](const Expression& v) { return v >= 0.0; }).value());
}

}  // namespace drake
