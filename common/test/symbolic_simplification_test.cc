#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

class SymbolicUnificationTest : public ::testing::Test {
 protected:
  // Provides common variables that are used by the following tests.
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable a_{"a"};
  const Variable b_{"b"};
  const Variable c_{"c"};
};

TEST_F(SymbolicUnificationTest, VariableSuccess) {
  // Rule: x => x + 1
  const RewritingRule rule{x_, x_ + 1};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), sin(a_) + 1);
  const Expression e2{a_ + b_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), a_ + b_ + 1);
}

TEST_F(SymbolicUnificationTest, VariableFailure) {
  // Rule: pow(x, x) => pow(x + 1, x + 2).
  const RewritingRule rule{pow(x_, x_), pow(x_ + 1, x_ + 2)};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match pow(a, b) with the above rule
  // since we can't have x ↦ a and x ↦ b at the same time.
  const Expression e{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, ConstantSuccess) {
  // Rule: 3 => 5.
  const RewritingRule rule{3, 5};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites 3 to 5.
  const Expression e{3};
  EXPECT_PRED2(ExprEqual, rewriter(e), 5);
}

TEST_F(SymbolicUnificationTest, ConstantFailure) {
  // Rule: 3 => 5.
  const RewritingRule rule{3, 5};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match 4 with with the above rule.
  const Expression e{4};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, AdditionSuccessNonZeroCoeff) {
  // Rule: 2 + x + y => 3 + 4x + 5y.
  const RewritingRule rule{2 + x_ + y_, 3 + 4 * x_ + 5 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites 2 + 2a + 3b to 3 + 4(2a) + 5(3b).
  const Expression e1{2 + 2 * a_ + 3 * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), 3 + 8 * a_ + 15 * b_);
  // It rewrites 2 + 2a + 3b + 5c to 3 + 4(2a) + 5(3b + 5c).
  const Expression e2{2 + 2 * a_ + 3 * b_ + 5 * c_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), 3 + 4 * 2 * a_ + 5 * (3 * b_ + 5 * c_));
}

TEST_F(SymbolicUnificationTest, AdditionSuccessZeroCoeff) {
  // Rule: x + y => 2x + 3y
  const RewritingRule rule{x_ + y_, 2 * x_ + 3 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites a + b => 2a + 3b
  const Expression e1{a_ + b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), 2 * a_ + 3 * b_);
  // It rewrites 1 + 2a => 2 + 6a
  const Expression e2{1 + 2 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), 2 + 6 * a_);
}

TEST_F(SymbolicUnificationTest, AdditionFailureNonZeroCoeff) {
  // Rule: 2 + sin(x) + cos(y) => 3 + 4x + 5y.
  const RewritingRule rule{2 + sin(x_) + cos(y_), 3 + 4 * x_ + 5 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with with the above rule (constant
  // mismatched).
  const Expression e1{5 + sin(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);

  // Fails to match the following with with the above rule (length).
  const Expression e2{2 + sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);

  // Fails to match the following with with the above rule (sin != cos).
  const Expression e3{2 + cos(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);

  // Fails to match the following with with the above rule (cos != tan).
  const Expression e4{2 + sin(a_) + tan(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e4), e4 /* no change */);

  // Fails to match the following with with the above rule (coefficient
  // mismatched).
  const Expression e5{sin(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e5), e5 /* no change */);
}

TEST_F(SymbolicUnificationTest, AdditionFailureZeroCoeff) {
  // Rule: sin(x) + cos(y) + sin(z) => x + y + z.
  const RewritingRule rule{sin(x_) + cos(y_) + sin(z_), x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with with the above rule (not addition).
  const Expression e1{cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);

  // Fails to match the following with with the above rule (length).
  const Expression e2{3 + cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);

  // Fails to match the following with with the above rule (sin(x) vs 1).
  const Expression e3{1 + log(a_) + abs(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);

  // Fails to match the following with with the above rule (sin != tan).
  const Expression e4{sin(b_) + cos(a_) + tan(c_)};
  EXPECT_PRED2(ExprEqual, rewriter(e4), e4 /* no change */);
}

TEST_F(SymbolicUnificationTest, UnaryMinus) {
  // -x => x.
  const RewritingRule rule{-x_, x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{-a_};
  const Expression expected1{a_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), expected1);
  const Expression e2{-2 * a_};
  const Expression expected2{2 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), expected2);
}

TEST_F(SymbolicUnificationTest, MultiplicationSuccess1) {
  // Rule: x * y => x + y
  const RewritingRule rule{x_ * y_, x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites a * b to a + b.
  const Expression e1{a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_);
  // It rewrites 3 * a to 3 + a.
  const Expression e2{3 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), 3 + a_);
}

TEST_F(SymbolicUnificationTest, MultiplicationSuccess2) {
  // Rule: x * pow(sin(y), 2) * cos(z)
  //    => x * pow(sin(y), 3) * tan(z).
  const RewritingRule rule{x_ * pow(sin(y_), 2) * cos(z_),
                           x_ * pow(sin(y_), 3) * tan(z_)};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites a*sin²(y)cos(z) to a sin³(y)tan(z).
  const Expression e{a_ * pow(sin(b_), 2) * cos(c_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), a_ * pow(sin(b_), 3) * tan(c_));
}

TEST_F(SymbolicUnificationTest, MultiplicationSuccess3) {
  // Rule: 2 * x * y => x + y.
  const RewritingRule rule{2 * x_ * y_, x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites 2 * a * b to a + b.
  const Expression e{2 * a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e), a_ + b_);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure1) {
  // Rule: 3pow(sin(x), cos(y))tan(z)
  //    => 4pow(x, y)pow(2, z)
  const RewritingRule rule{3 * pow(sin(x_), cos(y_)) * tan(z_),
                           4 * pow(x_, y_) * pow(2, z_)};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with with the above rule.
  const Expression e{pow(cos(a_), c_) * cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure2) {
  // Rule: sin(x) * cos(y) * tan(z)
  //    => x + y + z
  const RewritingRule rule{sin(x_) * cos(y_) * tan(z_), x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with with the above rule.
  const Expression e1{sin(a_) * acos(b_) * tan(c_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);
  // Fails to match the following with with the above rule.
  const Expression e2{a_ * sin(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
  // Fails to match the following with with the above rule.
  const Expression e3{5 * cos(b_) * tan(z_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);
  // Fails to match the following with with the above rule.
  const Expression e4{5};
  EXPECT_PRED2(ExprEqual, rewriter(e4), e4 /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure3) {
  // Rule: 2 * x * y * z => x + y + z.
  const RewritingRule rule{2 * x_ * y_ * z_, x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with with the above rule (coefficients).
  const Expression e1{3 * a_ * b_ * c_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);
  // Fails to match the following with with the above rule (length).
  const Expression e2{3 * a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure4) {
  // Rule: x * y * z => x + y + z.
  const RewritingRule rule{x_ * y_ * z_, x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with with the above rule.
  const Expression e{3 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, Pow) {
  // Rule: pow(x, y) => x + y
  const RewritingRule rule{pow(x_, y_), x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_);
  const Expression e2{atan2(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Division) {
  // Rule: x / y => x + y
  const RewritingRule rule{x_ / y_, x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{a_ / b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_);
  const Expression e2{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Abs) {
  // Rule: abs(x) => x
  const RewritingRule rule{abs(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{abs(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Log) {
  // Rule: log(x) => x
  const RewritingRule rule{log(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{log(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Exp) {
  // Rule: exp(x) => x
  const RewritingRule rule{exp(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{exp(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Sqrt) {
  // Rule: sqrt(x) => x
  const RewritingRule rule{sqrt(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{sqrt(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Sin) {
  // Rule: sin(x) => x
  const RewritingRule rule{sin(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Cos) {
  // Rule: cos(x) => x
  const RewritingRule rule{cos(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Tan) {
  // Rule: tan(x) => x
  const RewritingRule rule{tan(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{tan(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Asin) {
  // Rule: asin(x) => x
  const RewritingRule rule{asin(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{asin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Acos) {
  // Rule: acos(x) => x
  const RewritingRule rule{acos(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{acos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Atan) {
  // Rule: atan(x) => x
  const RewritingRule rule{atan(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{atan(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Atan2) {
  // Rule: atan2(x, y) => x + 2y
  const RewritingRule rule{atan2(x_, y_), x_ + 2 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{atan2(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + 2 * b_);
  const Expression e2{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Sinh) {
  // Rule: sinh(x) => x
  const RewritingRule rule{sinh(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{sinh(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Cosh) {
  // Rule: cosh(x) => x
  const RewritingRule rule{cosh(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{cosh(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Tanh) {
  // Rule: tanh(x) => x
  const RewritingRule rule{tanh(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{tanh(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Min) {
  // Rule: min(x, y) => x + 2y
  const RewritingRule rule{min(x_, y_), x_ + 2 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{min(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + 2 * b_);
  const Expression e2{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Max) {
  // Rule: max(x, y) => x + 2y
  const RewritingRule rule{max(x_, y_), x_ + 2 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{max(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + 2 * b_);
  const Expression e2{pow(a_, b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Ceil) {
  // Rule: ceil(x) => x
  const RewritingRule rule{ceil(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{ceil(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, Floor) {
  // Rule: floor(x) => x
  const RewritingRule rule{floor(x_), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e1{floor(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, IfThenElse) {
  // Not supported.
  const RewritingRule rule{if_then_else(x_ > y_, x_, y_), x_ + 2 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e{if_then_else(a_ > b_, a_, b_)};
  EXPECT_THROW(rewriter(e), std::runtime_error);
}

TEST_F(SymbolicUnificationTest, UninterpretedFunction) {
  // Not supported.
  const RewritingRule rule{uninterpreted_function("uf", {x_}), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e{uninterpreted_function("uf", {a_})};
  EXPECT_THROW(rewriter(e), std::runtime_error);
}

TEST_F(SymbolicUnificationTest, CheckCommonTrigIdentities1) {
  // Rule: sin²(x) + cos²(y) => 1.
  const RewritingRule rule{pow(sin(x_), 2) + pow(cos(y_), 2), 1};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites cos²(a + b) + sin²(a + b) to 1.0.
  const Expression e{pow(cos(a_ + b_), 2) + pow(sin(a_ + b_), 2)};
  EXPECT_PRED2(ExprEqual, rewriter(e), 1.0);
}

TEST_F(SymbolicUnificationTest, CheckCommonTrigIdentities2) {
  // Rule: sin(x + y) => sin(x)cos(y) + cos(x)sin(y).
  const RewritingRule rule{sin(x_ + y_), sin(x_) * cos(y_) + cos(x_) * sin(y_)};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites sin(2a + 3b) = sin(2a)cos(3b) + cos(3b)sin(2a).
  const Expression e{sin(2 * a_ + 3 * b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e),
               sin(2 * a_) * cos(3 * b_) + cos(2 * a_) * sin(3 * b_));
}

TEST_F(SymbolicUnificationTest, CheckCommonTrigIdentities3) {
  // Rule: tan(x - y) => (tan(x) - tan(y)) / (1 + tan(x)*tan(y)).
  const RewritingRule rule{tan(x_ - y_),
                           (tan(x_) - tan(y_)) / (1 + tan(x_) * tan(y_))};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites tan(2a - 3b) = (tan(2a) - tan(3b)) / (1 + tan(2a)*tan(3b)).
  const Expression e{tan(2 * a_ - 3 * b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e),
               (tan(2 * a_) - tan(3 * b_)) / (1 + tan(2 * a_) * tan(3 * b_)));
}

TEST_F(SymbolicUnificationTest, CheckCommonTrigIdentities4) {
  // Rule: cos(x) - cos(y) => -2sin((x + y)/2)sin((x - y)/2).
  const RewritingRule rule{cos(x_) - cos(y_),
                           -2 * sin((x_ + y_) / 2) * sin((x_ - y_) / 2)};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites cos(2a) - cos(4a) => -2sin((2a + 4a)/2)sin((2a - 4a) /2).
  const Expression e{cos(2 * a_) - cos(4 * a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), -2 * sin(6 * a_ / 2) * sin(-2 * a_ / 2));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
