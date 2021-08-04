#include "drake/common/symbolic_simplification.h"

#include <functional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::function;
using std::runtime_error;

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

  const Expression e1{a_ + b_};
  // The `rewriter` matches the whole `a + b` with the LHS of the rule, `x`, and
  // generates a substitution, {x ↦ a + b}. Then, it applies the substitution to
  // the RHS of the rule, `x + 1`, resulting in `a + b + 1`.
  //
  // Note that this is *not* a congruence-rewriter which would traverse the
  // expression `a + b`, rewrite `a` to `a + 1` and `b` to `b + 1`, and return
  // `(a + 1) + (b + 1)`, which is `a + b + 2`.
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_ + 1);
  const Expression e2{sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), sin(a_) + 1);
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
  // Fails to match 4 with the above rule.
  const Expression e{4};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, AdditionSuccessNonZeroCoeff) {
  // Rule: 2 + x + y => 3 + 4x + 5y.
  const RewritingRule rule{2 + x_ + y_, 3 + 4 * x_ + 5 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites 2 + 2a + 3b to 3 + 4(2a) + 5(3b).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e1{2 + 2 * a_ + 3 * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), 3 + 8 * a_ + 15 * b_);
  // It rewrites 2 + 2a + 3b + 5c to 3 + 4(2a) + 5(3b + 5c).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e2{2 + 2 * a_ + 3 * b_ + 5 * c_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), 3 + 4 * 2 * a_ + 5 * (3 * b_ + 5 * c_));
}

// This is an example of the Case 1 in the description of
// UnificationVisitor::VisitAddition method.
TEST_F(SymbolicUnificationTest, AdditionSuccessZeroCoeff) {
  // Rule: x + y => 2x + 3y
  const RewritingRule rule{x_ + y_, 2 * x_ + 3 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites a + b => 2a + 3b
  // Example of Case 1 in UnificationVisitor::VisitAddition.
  const Expression e1{a_ + b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), 2 * a_ + 3 * b_);
  // It rewrites 1 + 2a => 2 + 6a
  // Example of Case 2 in UnificationVisitor::VisitAddition.
  const Expression e2{1 + 2 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), 2 + 6 * a_);
}

TEST_F(SymbolicUnificationTest, AdditionFailureNonZeroCoeff) {
  // Rule: 2 + sin(x) + cos(y) => 3 + 4x + 5y.
  const RewritingRule rule{2 + sin(x_) + cos(y_), 3 + 4 * x_ + 5 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with the above rule (constant
  // mismatched).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e1{5 + sin(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);

  // Fails to match the following with the above rule (length).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e2{2 + sin(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);

  // Fails to match the following with the above rule (sin != cos).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e3{2 + cos(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);

  // Fails to match the following with the above rule (cos != tan).
  // Example of Case 4 in UnificationVisitor::VisitAddition.
  const Expression e4{2 + sin(a_) + tan(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e4), e4 /* no change */);

  // Fails to match the following with the above rule (coefficient
  // mismatched).
  // Example of Case 3 in UnificationVisitor::VisitAddition.
  const Expression e5{sin(a_) + cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e5), e5 /* no change */);
}

TEST_F(SymbolicUnificationTest, AdditionFailureZeroCoeff) {
  // Rule: sin(x) + cos(y) + sin(z) => x + y + z.
  const RewritingRule rule{sin(x_) + cos(y_) + sin(z_), x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with the above rule (not addition).
  const Expression e1{cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);

  // Fails to match the following with the above rule (length).
  // Example of Case 2 in UnificationVisitor::VisitAddition.
  const Expression e2{3 + cos(a_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);

  // Fails to match the following with the above rule (sin(x) vs 1).
  // Example of Case 2 in UnificationVisitor::VisitAddition.
  const Expression e3{1 + log(a_) + abs(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);

  // Fails to match the following with the above rule (sin != tan).
  // Example of Case 1 in UnificationVisitor::VisitAddition.
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
  // Example of Case 1 in UnificationVisitor::VisitMultiplication.
  const Expression e1{a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_);
  // It rewrites 3 * a to 3 + a.
  // Example of Case 2 in UnificationVisitor::VisitMultiplication.
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
  // Example of Case 1 in UnificationVisitor::VisitMultiplication.
  const Expression e{a_ * pow(sin(b_), 2) * cos(c_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), a_ * pow(sin(b_), 3) * tan(c_));
}

TEST_F(SymbolicUnificationTest, MultiplicationSuccess3) {
  // Rule: 2 * x * y => x + y.
  const RewritingRule rule{2 * x_ * y_, x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // It rewrites 2 * a * b to a + b.
  // Example of Case 4 in UnificationVisitor::VisitMultiplication.
  const Expression e{2 * a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e), a_ + b_);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure1) {
  // Rule: 3pow(sin(x), cos(y))tan(z)
  //    => 4pow(x, y)pow(2, z)
  const RewritingRule rule{3 * pow(sin(x_), cos(y_)) * tan(z_),
                           4 * pow(x_, y_) * pow(2, z_)};
  const Rewriter rewriter = MakeRuleRewriter(rule);

  // Fails to match the following with the above rule.
  // Example of Case 3 in UnificationVisitor::VisitMultiplication.
  const Expression e{pow(cos(a_), c_) * cos(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure2) {
  // Rule: sin(x) * cos(y) * tan(z)
  //    => x + y + z
  const RewritingRule rule{sin(x_) * cos(y_) * tan(z_), x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with the above rule.
  // Example of Case 1 in UnificationVisitor::VisitMultiplication.
  const Expression e1{sin(a_) * acos(b_) * tan(c_)};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);
  // Fails to match the following with the above rule.
  // Example of Case 1 in UnificationVisitor::VisitMultiplication.
  const Expression e2{a_ * sin(b_)};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
  // Fails to match the following with the above rule.
  // Example of Case 2 in UnificationVisitor::VisitMultiplication.
  const Expression e3{5 * cos(b_) * tan(z_)};
  EXPECT_PRED2(ExprEqual, rewriter(e3), e3 /* no change */);
  // Fails to match the following with the above rule.
  const Expression e4{5};
  EXPECT_PRED2(ExprEqual, rewriter(e4), e4 /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure3) {
  // Rule: 2 * x * y * z => x + y + z.
  const RewritingRule rule{2 * x_ * y_ * z_, x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with the above rule (coefficients).
  // Example of Case 4 in UnificationVisitor::VisitMultiplication.
  const Expression e1{3 * a_ * b_ * c_};
  EXPECT_PRED2(ExprEqual, rewriter(e1), e1 /* no change */);
  // Fails to match the following with the above rule (length).
  // Example of Case 4 in UnificationVisitor::VisitMultiplication.
  const Expression e2{3 * a_ * b_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

TEST_F(SymbolicUnificationTest, MultiplicationFailure4) {
  // Rule: x * y * z => x + y + z.
  const RewritingRule rule{x_ * y_ * z_, x_ + y_ + z_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  // Fails to match the following with the above rule.
  // Example of Case 2 in UnificationVisitor::VisitMultiplication.
  const Expression e{3 * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e), e /* no change */);
}

// https://github.com/google/googletest/issues/1610
enum UnaryTestOp {
  Abs, Log, Exp, Sqrt, Sin, Cos, Tan, Asin, Acos, Atan, Sinh, Cosh, Tanh, Ceil,
  Floor
};

std::function<Expression(const Variable& x)>UnaryOpToFunction(UnaryTestOp op) {
  switch (op) {
    case Abs:   return [](const Variable& x) { return abs(x); };
    case Log:   return [](const Variable& x) { return log(x); };
    case Exp:   return [](const Variable& x) { return exp(x); };
    case Sqrt:  return [](const Variable& x) { return sqrt(x); };
    case Sin:   return [](const Variable& x) { return sin(x); };
    case Cos:   return [](const Variable& x) { return cos(x); };
    case Tan:   return [](const Variable& x) { return tan(x); };
    case Asin:  return [](const Variable& x) { return asin(x); };
    case Acos:  return [](const Variable& x) { return acos(x); };
    case Atan:  return [](const Variable& x) { return atan(x); };
    case Sinh:  return [](const Variable& x) { return sinh(x); };
    case Cosh:  return [](const Variable& x) { return cosh(x); };
    case Tanh:  return [](const Variable& x) { return tanh(x); };
    case Ceil:  return [](const Variable& x) { return ceil(x); };
    case Floor: return [](const Variable& x) { return floor(x); };
  }
  DRAKE_UNREACHABLE();
}

class SymbolicUnificationTestUnary
    : public ::testing::TestWithParam<UnaryTestOp> {
 protected:
  const Variable x_{"x"};
  const Variable a_{"a"};
};

TEST_P(SymbolicUnificationTestUnary, Check) {
  const Expression& lhs = UnaryOpToFunction(GetParam())(x_);
  const RewritingRule rule{lhs, x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression& e1 = UnaryOpToFunction(GetParam())(a_);
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_);
  const Expression e2{a_ * a_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

INSTANTIATE_TEST_SUITE_P(
    UnaryCases, SymbolicUnificationTestUnary,
    ::testing::Values(
    Abs, Log, Exp, Sqrt, Sin, Cos, Tan, Asin, Acos, Atan, Sinh, Cosh, Tanh,
    Ceil, Floor
));

// https://github.com/google/googletest/issues/1610
enum BinaryTestOp {
  Pow, Div, Min, Max, Atan2
};

std::function<Expression(const Variable&, const Variable&)>
    BinaryOpToFunction(BinaryTestOp op) {
  switch (op) {
    case Pow:
      return [](const Variable& x, const Variable& y) { return pow(x, y); };

    case Div:
      return [](const Variable& x, const Variable& y) { return x / y; };

    case Min:
      return [](const Variable& x, const Variable& y) { return min(x, y); };

    case Max:
      return [](const Variable& x, const Variable& y) { return max(x, y); };

    case Atan2:
      return [](const Variable& x, const Variable& y) { return atan2(x, y); };
  }
  // Should not be reachable.
  DRAKE_UNREACHABLE();
}

class SymbolicUnificationTestBinary
    : public ::testing::TestWithParam<BinaryTestOp> {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable a_{"a"};
  const Variable b_{"b"};
};

TEST_P(SymbolicUnificationTestBinary, Check) {
  const Expression& lhs = BinaryOpToFunction(GetParam())(x_, y_);
  const RewritingRule rule{lhs, x_ + y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression& e1 = BinaryOpToFunction(GetParam())(a_, b_);
  EXPECT_PRED2(ExprEqual, rewriter(e1), a_ + b_);
  const Expression e2{a_ + b_};
  EXPECT_PRED2(ExprEqual, rewriter(e2), e2 /* no change */);
}

INSTANTIATE_TEST_SUITE_P(
    BinaryCases, SymbolicUnificationTestBinary,
    ::testing::Values(Pow, Div, Min, Max, Atan2));

TEST_F(SymbolicUnificationTest, IfThenElse) {
  // Not supported.
  const RewritingRule rule{if_then_else(x_ > y_, x_, y_), x_ + 2 * y_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e{if_then_else(a_ > b_, a_, b_)};
  EXPECT_THROW(rewriter(e), runtime_error);
}

TEST_F(SymbolicUnificationTest, UninterpretedFunction) {
  // Not supported.
  const RewritingRule rule{uninterpreted_function("uf", {x_}), x_};
  const Rewriter rewriter = MakeRuleRewriter(rule);
  const Expression e{uninterpreted_function("uf", {a_})};
  EXPECT_THROW(rewriter(e), runtime_error);
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
