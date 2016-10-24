#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

using std::domain_error;
using std::equal_to;
using std::runtime_error;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

// Provides common variables that are used by the following tests.
class SymbolicExpressionTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const Expression x_plus_y_{x_ + y_};

  const Expression x_plus_z_{x_ + z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{3.141592};
  const Expression neg_pi_{-3.141592};
  const Expression e_{2.718};

  const Expression c1_{0.0};
  const Expression c2_{1.0};
  const Expression c3_{3.14159};
  const Expression c4_{-2.718};
};

TEST_F(SymbolicExpressionTest, Variable) {
  EXPECT_EQ(x_.to_string(), var_x_.get_name());
  EXPECT_EQ(y_.to_string(), var_y_.get_name());
  EXPECT_EQ(z_.to_string(), var_z_.get_name());
  EXPECT_TRUE(x_.EqualTo(x_));
  EXPECT_FALSE(x_.EqualTo(y_));
  EXPECT_FALSE(x_.EqualTo(z_));
  EXPECT_FALSE(y_.EqualTo(x_));
  EXPECT_TRUE(y_.EqualTo(y_));
  EXPECT_FALSE(y_.EqualTo(z_));
  EXPECT_FALSE(z_.EqualTo(x_));
  EXPECT_FALSE(z_.EqualTo(y_));
  EXPECT_TRUE(z_.EqualTo(z_));
}

TEST_F(SymbolicExpressionTest, Constant) {
  EXPECT_EQ(c1_.Evaluate(), 0);
  EXPECT_EQ(c2_.Evaluate(), 1);
  EXPECT_EQ(c3_.Evaluate(), 3.14159);
  EXPECT_EQ(c4_.Evaluate(), -2.718);
  EXPECT_THROW(Expression{NAN}, runtime_error);
}

TEST_F(SymbolicExpressionTest, StaticConstant) {
  EXPECT_DOUBLE_EQ(Expression::Zero().Evaluate(), 0.0);
  EXPECT_DOUBLE_EQ(Expression::One().Evaluate(), 1.0);
  EXPECT_NEAR(Expression::Pi().Evaluate(), 3.141592, 0.000001);
  EXPECT_NEAR(Expression::E().Evaluate(), 2.718281828, 0.000000001);
}

TEST_F(SymbolicExpressionTest, Hash) {
  Expression x{var_x_};
  const Expression x_prime(x);
  EXPECT_EQ(x.get_hash(), x_prime.get_hash());
  x++;
  EXPECT_NE(x.get_hash(), x_prime.get_hash());
}

TEST_F(SymbolicExpressionTest, HashBinary) {
  const Expression e1{x_plus_y_ + x_plus_z_};
  const Expression e2{x_plus_y_ - x_plus_z_};
  const Expression e3{x_plus_y_ * x_plus_z_};
  const Expression e4{x_plus_y_ / x_plus_z_};
  const Expression e5{pow(x_plus_y_, x_plus_z_)};
  const Expression e6{atan2(x_plus_y_, x_plus_z_)};
  const Expression e7{min(x_plus_y_, x_plus_z_)};
  const Expression e8{max(x_plus_y_, x_plus_z_)};

  // e1, ..., e8 share the same sub-expressions, but their hash values should be
  // distinct.
  unordered_set<size_t> hash_set;
  const vector<Expression> exprs{e1, e2, e3, e4, e5, e6, e7, e8};
  for (auto const& e : exprs) {
    hash_set.insert(e.get_hash());
  }
  EXPECT_EQ(hash_set.size(), exprs.size());
}

TEST_F(SymbolicExpressionTest, HashUnary) {
  const Expression e0{log(x_plus_y_)};
  const Expression e1{abs(x_plus_y_)};
  const Expression e2{exp(x_plus_y_)};
  const Expression e3{sqrt(x_plus_y_)};
  const Expression e4{sin(x_plus_y_)};
  const Expression e5{cos(x_plus_y_)};
  const Expression e6{tan(x_plus_y_)};
  const Expression e7{asin(x_plus_y_)};
  const Expression e8{acos(x_plus_y_)};
  const Expression e9{atan(x_plus_y_)};
  const Expression e10{sinh(x_plus_y_)};
  const Expression e11{cosh(x_plus_y_)};
  const Expression e12{tanh(x_plus_y_)};

  // e0, ..., e12 share the same sub-expression, but their hash values should be
  // distinct.
  unordered_set<size_t> hash_set;
  const vector<Expression> exprs{e0, e1, e2, e3,  e4,  e5, e6,
                                 e7, e8, e9, e10, e11, e12};
  for (auto const& e : exprs) {
    hash_set.insert(e.get_hash());
  }
  EXPECT_EQ(hash_set.size(), exprs.size());
}

TEST_F(SymbolicExpressionTest, UnaryMinus) {
  EXPECT_FALSE(c3_.EqualTo(-c3_));
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), -(-c3_).Evaluate());
  EXPECT_TRUE(c3_.EqualTo(-(-c3_)));
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), (-(-c3_)).Evaluate());
  const Expression e{x_ + y_};
  const Environment env{{var_x_, 1.0}, {var_y_, 2.0}};
  // (x + y) and -(-(x + y)) are structurally different
  EXPECT_FALSE(e.EqualTo(-(-e)));
  // but their evaluations should be the same.
  EXPECT_DOUBLE_EQ(e.Evaluate(env), (-(-e)).Evaluate(env));
}

TEST_F(SymbolicExpressionTest, Add1) {
  EXPECT_TRUE((c3_ + zero_).EqualTo(c3_));
  EXPECT_EQ((c3_ + zero_).to_string(), c3_.to_string());
  EXPECT_TRUE((zero_ + c3_).EqualTo(c3_));
  EXPECT_EQ((zero_ + c3_).to_string(), c3_.to_string());
  EXPECT_TRUE((0.0 + c3_).EqualTo(c3_));
  EXPECT_EQ((0.0 + c3_).to_string(), c3_.to_string());
  EXPECT_TRUE((c3_ + 0.0).EqualTo(c3_));
  EXPECT_EQ((c3_ + 0.0).to_string(), c3_.to_string());
  EXPECT_TRUE(equal_to<Expression>{}(c3_ + c4_, Expression{3.14159 + -2.718}));
  EXPECT_EQ((c3_ + c4_).to_string(), Expression{3.14159 + -2.718}.to_string());
  EXPECT_TRUE(equal_to<Expression>{}(c3_ + x_, 3.14159 + x_));
  EXPECT_EQ((c3_ + x_).to_string(), (3.14159 + x_).to_string());
  EXPECT_TRUE(equal_to<Expression>{}(x_ + c3_, x_ + 3.14159));
  EXPECT_EQ((x_ + c3_).to_string(), (x_ + 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Add2) {
  Expression e1{x_ + y_};
  Expression e2{e1 + e1};
  EXPECT_EQ(e1.to_string(), "(x + y)");
  EXPECT_EQ(e2.to_string(), "((x + y) + (x + y))");
  e1 += z_;
  EXPECT_EQ(e1.to_string(), "((x + y) + z)");
  EXPECT_EQ(e2.to_string(), "((x + y) + (x + y))");  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Inc1) {
  // Prefix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};
  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_TRUE(x++.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime++));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

TEST_F(SymbolicExpressionTest, Inc2) {
  // Postfix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_FALSE((++x).EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_TRUE(x.EqualTo(++x_prime));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

TEST_F(SymbolicExpressionTest, Inc3) {
  // Pre/Post increments
  Expression c1{3.1415};
  EXPECT_DOUBLE_EQ((c1++).Evaluate(), 3.1415);
  EXPECT_DOUBLE_EQ(c1.Evaluate(), 3.1415 + 1.0);

  Expression c2{3.1415};
  EXPECT_DOUBLE_EQ((++c2).Evaluate(), 3.1415 + 1.0);
  EXPECT_DOUBLE_EQ(c2.Evaluate(), 3.1415 + 1.0);
}

TEST_F(SymbolicExpressionTest, Sub1) {
  EXPECT_EQ((c3_ - zero_).to_string(), c3_.to_string());
  EXPECT_EQ((zero_ - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_EQ((0.0 - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_EQ((0.0 - c3_).to_string(), (-1 * c3_).to_string());
  EXPECT_EQ((c3_ - 0.0).to_string(), c3_.to_string());
  EXPECT_EQ((c3_ - c4_).to_string(), Expression{3.14159 - -2.718}.to_string());
  EXPECT_EQ((c3_ - x_).to_string(), (3.14159 - x_).to_string());
  EXPECT_EQ((x_ - c3_).to_string(), (x_ - 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Sub2) {
  Expression e1{x_ - y_};
  const Expression e2{x_ - z_};
  const Expression e3{e1 - e2};
  EXPECT_EQ(e1.to_string(), "(x - y)");
  EXPECT_EQ(e2.to_string(), "(x - z)");
  EXPECT_EQ(e3.to_string(), "((x - y) - (x - z))");
  e1 -= z_;
  EXPECT_EQ(e1.to_string(), "((x - y) - z)");
  EXPECT_EQ(e3.to_string(), "((x - y) - (x - z))");  // e3 doesn't change.
}

TEST_F(SymbolicExpressionTest, Sub3) {
  const Expression e1{x_ - y_};
  const Expression e2{x_ - y_};
  const Expression e3{e1 - e2};
  EXPECT_EQ(e1.to_string(), "(x - y)");
  EXPECT_EQ(e2.to_string(), "(x - y)");
  EXPECT_EQ(e3.to_string(), "0");  // simplified
}

TEST_F(SymbolicExpressionTest, Dec1) {
  // Postfix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_TRUE(x--.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime--));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

TEST_F(SymbolicExpressionTest, Dec2) {
  // Prefix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_FALSE((--x).EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_TRUE(x.EqualTo(--x_prime));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

TEST_F(SymbolicExpressionTest, Dec3) {
  // Pre/Postfix decrements.
  Expression c1{3.1415};
  EXPECT_DOUBLE_EQ((c1--).Evaluate(), 3.1415);
  EXPECT_DOUBLE_EQ(c1.Evaluate(), 3.1415 - 1.0);

  Expression c2{3.1415};
  EXPECT_DOUBLE_EQ((--c2).Evaluate(), 3.1415 - 1.0);
  EXPECT_DOUBLE_EQ(c2.Evaluate(), 3.1415 - 1.0);
}

TEST_F(SymbolicExpressionTest, MUL1) {
  EXPECT_EQ((c3_ * zero_).to_string(), zero_.to_string());
  EXPECT_EQ((zero_ * c3_).to_string(), zero_.to_string());
  EXPECT_EQ((c3_ * 0.0).to_string(), zero_.to_string());
  EXPECT_EQ((0.0 * c3_).to_string(), zero_.to_string());

  EXPECT_EQ((c3_ * one_).to_string(), c3_.to_string());
  EXPECT_EQ((one_ * c3_).to_string(), c3_.to_string());
  EXPECT_EQ((1.0 * c3_).to_string(), c3_.to_string());
  EXPECT_EQ((c3_ * 1.0).to_string(), c3_.to_string());

  EXPECT_EQ((c3_ * c4_).to_string(), Expression{3.14159 * -2.718}.to_string());
  EXPECT_EQ((c3_ * x_).to_string(), (3.14159 * x_).to_string());
  EXPECT_EQ((x_ * c3_).to_string(), (x_ * 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, MUL2) {
  Expression e1{x_ * y_};
  Expression e2{e1 * e1};
  EXPECT_EQ(e1.to_string(), "(x * y)");
  EXPECT_EQ(e2.to_string(), "((x * y) * (x * y))");
  e1 *= z_;
  EXPECT_EQ(e1.to_string(), "((x * y) * z)");
  EXPECT_EQ(e2.to_string(), "((x * y) * (x * y))");  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Div1) {
  EXPECT_THROW(c3_ / zero_, runtime_error);
  EXPECT_EQ((zero_ / c3_).to_string(), zero_.to_string());
  EXPECT_THROW(c3_ / 0.0, runtime_error);
  EXPECT_EQ((0.0 / c3_).to_string(), zero_.to_string());

  EXPECT_EQ((c3_ / one_).to_string(), c3_.to_string());
  EXPECT_EQ((c3_ / 1.0).to_string(), c3_.to_string());

  EXPECT_EQ((c3_ / c4_).to_string(), Expression{3.14159 / -2.718}.to_string());
  EXPECT_EQ((c3_ / x_).to_string(), (3.14159 / x_).to_string());
  EXPECT_EQ((x_ / c3_).to_string(), (x_ / 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Div2) {
  Expression e1{x_ / y_};
  const Expression e2{x_ / z_};
  const Expression e3{e1 / e2};
  EXPECT_EQ(e1.to_string(), "(x / y)");
  EXPECT_EQ(e2.to_string(), "(x / z)");
  EXPECT_EQ(e3.to_string(), "((x / y) / (x / z))");
  e1 /= z_;
  EXPECT_EQ(e1.to_string(), "((x / y) / z)");
  EXPECT_EQ(e3.to_string(), "((x / y) / (x / z))");  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Div3) {
  const Expression e1{x_ / y_};
  const Expression e2{x_ / y_};
  const Expression e3{e1 / e2};
  EXPECT_EQ(e1.to_string(), "(x / y)");
  EXPECT_EQ(e2.to_string(), "(x / y)");
  EXPECT_EQ(e3.to_string(), "1");  // simplified
}

// This test checks whether symbolic::Expression is compatible with
// std::unordered_set.
GTEST_TEST(ExpressionTest, CompatibleWithUnorderedSet) {
  unordered_set<Expression> uset;
  uset.emplace(Expression{Variable{"a"}});
  uset.emplace(Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::unordered_map.
GTEST_TEST(ExpressionTest, CompatibleWithUnorderedMap) {
  unordered_map<Expression, Expression> umap;
  umap.emplace(Expression{Variable{"a"}}, Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::vector.
GTEST_TEST(ExpressionTest, CompatibleWithVector) {
  vector<Expression> vec;
  vec.push_back(Expression{123.0});
}

TEST_F(SymbolicExpressionTest, Log) {
  EXPECT_DOUBLE_EQ(log(pi_).Evaluate(), std::log(3.141592));
  EXPECT_DOUBLE_EQ(log(one_).Evaluate(), std::log(1.0));
  EXPECT_DOUBLE_EQ(log(zero_).Evaluate(), std::log(0.0));
  EXPECT_THROW(log(neg_one_).Evaluate(), domain_error);
  EXPECT_THROW(log(neg_pi_).Evaluate(), domain_error);
  const Expression e{log(x_ * y_ * pi_) + log(x_) + log(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::log(2 * 3.2 * 3.141592) + std::log(2) + std::log(3.2));
}

TEST_F(SymbolicExpressionTest, Abs) {
  EXPECT_DOUBLE_EQ(abs(pi_).Evaluate(), std::fabs(3.141592));
  EXPECT_DOUBLE_EQ(abs(one_).Evaluate(), std::fabs(1.0));
  EXPECT_DOUBLE_EQ(abs(zero_).Evaluate(), std::fabs(0.0));
  EXPECT_DOUBLE_EQ(abs(neg_one_).Evaluate(), std::fabs(-1.0));
  EXPECT_DOUBLE_EQ(abs(neg_pi_).Evaluate(), std::fabs(-3.141592));
  const Expression e{abs(x_ * y_ * pi_) + abs(x_) + abs(y_)};
  const Environment env{{var_x_, -2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::fabs(-2 * 3.2 * 3.141592) +
                                        std::fabs(-2.0) + std::fabs(3.2));
}

TEST_F(SymbolicExpressionTest, Exp) {
  EXPECT_DOUBLE_EQ(exp(pi_).Evaluate(), std::exp(3.141592));
  EXPECT_DOUBLE_EQ(exp(one_).Evaluate(), std::exp(1));
  EXPECT_DOUBLE_EQ(exp(zero_).Evaluate(), std::exp(0));
  EXPECT_DOUBLE_EQ(exp(neg_one_).Evaluate(), std::exp(-1));
  EXPECT_DOUBLE_EQ(exp(neg_pi_).Evaluate(), std::exp(-3.141592));

  const Expression e{exp(x_ * y_ * pi_) + exp(x_) + exp(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::exp(2 * 3.2 * 3.141592) +
                                        std::exp(2.0) + std::exp(3.2));
}

TEST_F(SymbolicExpressionTest, Sqrt) {
  EXPECT_DOUBLE_EQ(sqrt(pi_).Evaluate(), std::sqrt(3.141592));
  EXPECT_DOUBLE_EQ(sqrt(one_).Evaluate(), std::sqrt(1.0));
  EXPECT_DOUBLE_EQ(sqrt(zero_).Evaluate(), std::sqrt(0.0));
  EXPECT_THROW(sqrt(neg_one_).Evaluate(), domain_error);
  EXPECT_THROW(sqrt(neg_pi_).Evaluate(), domain_error);

  const Expression e{sqrt(x_ * y_ * pi_) + sqrt(x_) + sqrt(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::sqrt(2 * 3.2 * 3.141592) +
                                        std::sqrt(2.0) + std::sqrt(3.2));
}

TEST_F(SymbolicExpressionTest, Pow) {
  EXPECT_DOUBLE_EQ(pow(pi_, pi_).Evaluate(), std::pow(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(pow(pi_, one_).Evaluate(), std::pow(3.141592, 1));
  EXPECT_DOUBLE_EQ(pow(pi_, two_).Evaluate(), std::pow(3.141592, 2));
  EXPECT_DOUBLE_EQ(pow(pi_, zero_).Evaluate(), std::pow(3.141592, 0));
  EXPECT_DOUBLE_EQ(pow(pi_, neg_one_).Evaluate(), std::pow(3.141592, -1));
  EXPECT_DOUBLE_EQ(pow(pi_, neg_pi_).Evaluate(), std::pow(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(pow(one_, pi_).Evaluate(), std::pow(1, 3.141592));
  EXPECT_DOUBLE_EQ(pow(one_, one_).Evaluate(), std::pow(1, 1));
  EXPECT_DOUBLE_EQ(pow(one_, two_).Evaluate(), std::pow(1, 2));
  EXPECT_DOUBLE_EQ(pow(one_, zero_).Evaluate(), std::pow(1, 0));
  EXPECT_DOUBLE_EQ(pow(one_, neg_one_).Evaluate(), std::pow(1, -1));
  EXPECT_DOUBLE_EQ(pow(one_, neg_pi_).Evaluate(), std::pow(1, -3.141592));

  EXPECT_DOUBLE_EQ(pow(two_, pi_).Evaluate(), std::pow(2, 3.141592));
  EXPECT_DOUBLE_EQ(pow(two_, one_).Evaluate(), std::pow(2, 1));
  EXPECT_DOUBLE_EQ(pow(two_, two_).Evaluate(), std::pow(2, 2));
  EXPECT_DOUBLE_EQ(pow(two_, zero_).Evaluate(), std::pow(2, 0));
  EXPECT_DOUBLE_EQ(pow(two_, neg_one_).Evaluate(), std::pow(2, -1));
  EXPECT_DOUBLE_EQ(pow(two_, neg_pi_).Evaluate(), std::pow(2, -3.141592));

  EXPECT_DOUBLE_EQ(pow(zero_, pi_).Evaluate(), std::pow(0, 3.141592));
  EXPECT_DOUBLE_EQ(pow(zero_, one_).Evaluate(), std::pow(0, 1));
  EXPECT_DOUBLE_EQ(pow(zero_, two_).Evaluate(), std::pow(0, 2));
  EXPECT_DOUBLE_EQ(pow(zero_, zero_).Evaluate(), std::pow(0, 0));
  EXPECT_DOUBLE_EQ(pow(zero_, neg_one_).Evaluate(), std::pow(0, -1));
  EXPECT_DOUBLE_EQ(pow(zero_, neg_pi_).Evaluate(), std::pow(0, -3.141592));

  EXPECT_THROW(pow(neg_one_, pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_one_, one_).Evaluate(), std::pow(-1, 1));
  EXPECT_DOUBLE_EQ(pow(neg_one_, two_).Evaluate(), std::pow(-1, 2));
  EXPECT_DOUBLE_EQ(pow(neg_one_, zero_).Evaluate(), std::pow(-1, 0));
  EXPECT_DOUBLE_EQ(pow(neg_one_, neg_one_).Evaluate(), std::pow(-1, -1));
  EXPECT_THROW(pow(neg_one_, neg_pi_).Evaluate(), domain_error);

  EXPECT_THROW(pow(neg_pi_, pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_pi_, one_).Evaluate(), std::pow(-3.141592, 1));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, two_).Evaluate(), std::pow(-3.141592, 2));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, zero_).Evaluate(), std::pow(-3.141592, 0));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, neg_one_).Evaluate(), std::pow(-3.141592, -1));
  EXPECT_THROW(pow(neg_pi_, neg_pi_).Evaluate(), domain_error);

  const Expression e{pow(x_ * y_ * pi_, x_ + y_ + pi_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::pow(2 * 3.2 * 3.141592, 2 + 3.2 + 3.141592));
}

TEST_F(SymbolicExpressionTest, Sin) {
  EXPECT_DOUBLE_EQ(sin(pi_).Evaluate(), std::sin(3.141592));
  EXPECT_DOUBLE_EQ(sin(one_).Evaluate(), std::sin(1));
  EXPECT_DOUBLE_EQ(sin(two_).Evaluate(), std::sin(2));
  EXPECT_DOUBLE_EQ(sin(zero_).Evaluate(), std::sin(0));
  EXPECT_DOUBLE_EQ(sin(neg_one_).Evaluate(), std::sin(-1));
  EXPECT_DOUBLE_EQ(sin(neg_pi_).Evaluate(), std::sin(-3.141592));

  const Expression e{sin(x_ * y_ * pi_) + sin(x_) + sin(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::sin(2 * 3.2 * 3.141592) + std::sin(2) + std::sin(3.2));
}

TEST_F(SymbolicExpressionTest, Cos) {
  EXPECT_DOUBLE_EQ(cos(pi_).Evaluate(), std::cos(3.141592));
  EXPECT_DOUBLE_EQ(cos(one_).Evaluate(), std::cos(1));
  EXPECT_DOUBLE_EQ(cos(two_).Evaluate(), std::cos(2));
  EXPECT_DOUBLE_EQ(cos(zero_).Evaluate(), std::cos(0));
  EXPECT_DOUBLE_EQ(cos(neg_one_).Evaluate(), std::cos(-1));
  EXPECT_DOUBLE_EQ(cos(neg_pi_).Evaluate(), std::cos(-3.141592));
  const Expression e{cos(x_ * y_ * pi_) + cos(x_) + cos(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::cos(2 * 3.2 * 3.141592) + std::cos(2) + std::cos(3.2));
}

TEST_F(SymbolicExpressionTest, Tan) {
  EXPECT_DOUBLE_EQ(tan(pi_).Evaluate(), std::tan(3.141592));
  EXPECT_DOUBLE_EQ(tan(one_).Evaluate(), std::tan(1));
  EXPECT_DOUBLE_EQ(tan(two_).Evaluate(), std::tan(2));
  EXPECT_DOUBLE_EQ(tan(zero_).Evaluate(), std::tan(0));
  EXPECT_DOUBLE_EQ(tan(neg_one_).Evaluate(), std::tan(-1));
  EXPECT_DOUBLE_EQ(tan(neg_pi_).Evaluate(), std::tan(-3.141592));

  const Expression e{tan(x_ * y_ * pi_) + tan(x_) + tan(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::tan(2 * 3.2 * 3.141592) + std::tan(2) + std::tan(3.2));
}

TEST_F(SymbolicExpressionTest, Asin) {
  EXPECT_THROW(asin(pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(asin(one_).Evaluate(), std::asin(1));
  EXPECT_THROW(asin(two_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(asin(zero_).Evaluate(), std::asin(0));
  EXPECT_DOUBLE_EQ(asin(neg_one_).Evaluate(), std::asin(-1));
  EXPECT_THROW(asin(neg_pi_).Evaluate(), domain_error);

  const Expression e{asin(x_ * y_ * pi_) + asin(x_) + asin(y_)};
  const Environment env{{var_x_, 0.2}, {var_y_, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::asin(0.2 * 0.3 * 3.141592) +
                                        std::asin(0.2) + std::asin(0.3));
}

TEST_F(SymbolicExpressionTest, Acos) {
  EXPECT_THROW(acos(pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(acos(one_).Evaluate(), std::acos(1));
  EXPECT_THROW(acos(two_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(acos(zero_).Evaluate(), std::acos(0));
  EXPECT_DOUBLE_EQ(acos(neg_one_).Evaluate(), std::acos(-1));
  EXPECT_THROW(acos(neg_pi_).Evaluate(), domain_error);

  const Expression e{acos(x_ * y_ * pi_) + acos(x_) + acos(y_)};
  const Environment env{{var_x_, 0.2}, {var_y_, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::acos(0.2 * 0.3 * 3.141592) +
                                        std::acos(0.2) + std::acos(0.3));
}

TEST_F(SymbolicExpressionTest, Atan) {
  EXPECT_DOUBLE_EQ(atan(pi_).Evaluate(), std::atan(3.141592));
  EXPECT_DOUBLE_EQ(atan(one_).Evaluate(), std::atan(1));
  EXPECT_DOUBLE_EQ(atan(two_).Evaluate(), std::atan(2));
  EXPECT_DOUBLE_EQ(atan(zero_).Evaluate(), std::atan(0));
  EXPECT_DOUBLE_EQ(atan(neg_one_).Evaluate(), std::atan(-1));
  EXPECT_DOUBLE_EQ(atan(neg_pi_).Evaluate(), std::atan(-3.141592));

  const Expression e{atan(x_ * y_ * pi_) + atan(x_) + atan(y_)};
  const Environment env{{var_x_, 0.2}, {var_y_, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::atan(0.2 * 0.3 * 3.141592) +
                                        std::atan(0.2) + std::atan(0.3));
}

TEST_F(SymbolicExpressionTest, Atan2) {
  EXPECT_DOUBLE_EQ(atan2(pi_, pi_).Evaluate(), std::atan2(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(pi_, one_).Evaluate(), std::atan2(3.141592, 1));
  EXPECT_DOUBLE_EQ(atan2(pi_, two_).Evaluate(), std::atan2(3.141592, 2));
  EXPECT_DOUBLE_EQ(atan2(pi_, zero_).Evaluate(), std::atan2(3.141592, 0));
  EXPECT_DOUBLE_EQ(atan2(pi_, neg_one_).Evaluate(), std::atan2(3.141592, -1));
  EXPECT_DOUBLE_EQ(atan2(pi_, neg_pi_).Evaluate(),
                   std::atan2(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(one_, pi_).Evaluate(), std::atan2(1, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(one_, one_).Evaluate(), std::atan2(1, 1));
  EXPECT_DOUBLE_EQ(atan2(one_, two_).Evaluate(), std::atan2(1, 2));
  EXPECT_DOUBLE_EQ(atan2(one_, zero_).Evaluate(), std::atan2(1, 0));
  EXPECT_DOUBLE_EQ(atan2(one_, neg_one_).Evaluate(), std::atan2(1, -1));
  EXPECT_DOUBLE_EQ(atan2(one_, neg_pi_).Evaluate(), std::atan2(1, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(two_, pi_).Evaluate(), std::atan2(2, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(two_, one_).Evaluate(), std::atan2(2, 1));
  EXPECT_DOUBLE_EQ(atan2(two_, two_).Evaluate(), std::atan2(2, 2));
  EXPECT_DOUBLE_EQ(atan2(two_, zero_).Evaluate(), std::atan2(2, 0));
  EXPECT_DOUBLE_EQ(atan2(two_, neg_one_).Evaluate(), std::atan2(2, -1));
  EXPECT_DOUBLE_EQ(atan2(two_, neg_pi_).Evaluate(), std::atan2(2, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(zero_, pi_).Evaluate(), std::atan2(0, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(zero_, one_).Evaluate(), std::atan2(0, 1));
  EXPECT_DOUBLE_EQ(atan2(zero_, two_).Evaluate(), std::atan2(0, 2));
  EXPECT_DOUBLE_EQ(atan2(zero_, zero_).Evaluate(), std::atan2(0, 0));
  EXPECT_DOUBLE_EQ(atan2(zero_, neg_one_).Evaluate(), std::atan2(0, -1));
  EXPECT_DOUBLE_EQ(atan2(zero_, neg_pi_).Evaluate(), std::atan2(0, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(neg_one_, pi_).Evaluate(), std::atan2(-1, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, one_).Evaluate(), std::atan2(-1, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, two_).Evaluate(), std::atan2(-1, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, zero_).Evaluate(), std::atan2(-1, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, neg_one_).Evaluate(), std::atan2(-1, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, neg_pi_).Evaluate(),
                   std::atan2(-1, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(neg_pi_, pi_).Evaluate(),
                   std::atan2(-3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, one_).Evaluate(), std::atan2(-3.141592, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, two_).Evaluate(), std::atan2(-3.141592, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, zero_).Evaluate(), std::atan2(-3.141592, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, neg_one_).Evaluate(),
                   std::atan2(-3.141592, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, neg_pi_).Evaluate(),
                   std::atan2(-3.141592, -3.141592));

  const Expression e{atan2(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::atan2(2 * 3.2 * 3.141592, std::sin(2) + std::sin(3.2)));
}

TEST_F(SymbolicExpressionTest, Sinh) {
  EXPECT_DOUBLE_EQ(sinh(pi_).Evaluate(), std::sinh(3.141592));
  EXPECT_DOUBLE_EQ(sinh(one_).Evaluate(), std::sinh(1));
  EXPECT_DOUBLE_EQ(sinh(two_).Evaluate(), std::sinh(2));
  EXPECT_DOUBLE_EQ(sinh(zero_).Evaluate(), std::sinh(0));
  EXPECT_DOUBLE_EQ(sinh(neg_one_).Evaluate(), std::sinh(-1));
  EXPECT_DOUBLE_EQ(sinh(neg_pi_).Evaluate(), std::sinh(-3.141592));

  const Expression e{sinh(x_ * y_ * pi_) + sinh(x_) + sinh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::sinh(2 * 3.2 * 3.141592) +
                                        std::sinh(2) + std::sinh(3.2));
}

TEST_F(SymbolicExpressionTest, Cosh) {
  EXPECT_DOUBLE_EQ(cosh(pi_).Evaluate(), std::cosh(3.141592));
  EXPECT_DOUBLE_EQ(cosh(one_).Evaluate(), std::cosh(1));
  EXPECT_DOUBLE_EQ(cosh(two_).Evaluate(), std::cosh(2));
  EXPECT_DOUBLE_EQ(cosh(zero_).Evaluate(), std::cosh(0));
  EXPECT_DOUBLE_EQ(cosh(neg_one_).Evaluate(), std::cosh(-1));
  EXPECT_DOUBLE_EQ(cosh(neg_pi_).Evaluate(), std::cosh(-3.141592));

  const Expression e{cosh(x_ * y_ * pi_) + cosh(x_) + cosh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::cosh(2 * 3.2 * 3.141592) +
                                        std::cosh(2) + std::cosh(3.2));
}

TEST_F(SymbolicExpressionTest, Tanh) {
  EXPECT_DOUBLE_EQ(tanh(pi_).Evaluate(), std::tanh(3.141592));
  EXPECT_DOUBLE_EQ(tanh(one_).Evaluate(), std::tanh(1));
  EXPECT_DOUBLE_EQ(tanh(two_).Evaluate(), std::tanh(2));
  EXPECT_DOUBLE_EQ(tanh(zero_).Evaluate(), std::tanh(0));
  EXPECT_DOUBLE_EQ(tanh(neg_one_).Evaluate(), std::tanh(-1));
  EXPECT_DOUBLE_EQ(tanh(neg_pi_).Evaluate(), std::tanh(-3.141592));

  const Expression e{tanh(x_ * y_ * pi_) + tanh(x_) + tanh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::tanh(2 * 3.2 * 3.141592) +
                                        std::tanh(2) + std::tanh(3.2));
}

TEST_F(SymbolicExpressionTest, Min1) {
  // min(E, E) -> E
  EXPECT_TRUE(min(x_plus_y_, x_plus_y_).EqualTo(x_plus_y_));
}

TEST_F(SymbolicExpressionTest, Min2) {
  EXPECT_DOUBLE_EQ(min(pi_, pi_).Evaluate(), std::min(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(min(pi_, one_).Evaluate(), std::min(3.141592, 1.0));
  EXPECT_DOUBLE_EQ(min(pi_, two_).Evaluate(), std::min(3.141592, 2.0));
  EXPECT_DOUBLE_EQ(min(pi_, zero_).Evaluate(), std::min(3.141592, 0.0));
  EXPECT_DOUBLE_EQ(min(pi_, neg_one_).Evaluate(), std::min(3.141592, -1.0));
  EXPECT_DOUBLE_EQ(min(pi_, neg_pi_).Evaluate(), std::min(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(min(one_, pi_).Evaluate(), std::min(1.0, 3.141592));
  EXPECT_DOUBLE_EQ(min(one_, one_).Evaluate(), std::min(1.0, 1.0));
  EXPECT_DOUBLE_EQ(min(one_, two_).Evaluate(), std::min(1.0, 2.0));
  EXPECT_DOUBLE_EQ(min(one_, zero_).Evaluate(), std::min(1.0, 0.0));
  EXPECT_DOUBLE_EQ(min(one_, neg_one_).Evaluate(), std::min(1.0, -1.0));
  EXPECT_DOUBLE_EQ(min(one_, neg_pi_).Evaluate(), std::min(1.0, -3.141592));

  EXPECT_DOUBLE_EQ(min(two_, pi_).Evaluate(), std::min(2.0, 3.141592));
  EXPECT_DOUBLE_EQ(min(two_, one_).Evaluate(), std::min(2.0, 1.0));
  EXPECT_DOUBLE_EQ(min(two_, two_).Evaluate(), std::min(2.0, 2.0));
  EXPECT_DOUBLE_EQ(min(two_, zero_).Evaluate(), std::min(2.0, 0.0));
  EXPECT_DOUBLE_EQ(min(two_, neg_one_).Evaluate(), std::min(2.0, -1.0));
  EXPECT_DOUBLE_EQ(min(two_, neg_pi_).Evaluate(), std::min(2.0, -3.141592));

  EXPECT_DOUBLE_EQ(min(zero_, pi_).Evaluate(), std::min(0.0, 3.141592));
  EXPECT_DOUBLE_EQ(min(zero_, one_).Evaluate(), std::min(0.0, 1.0));
  EXPECT_DOUBLE_EQ(min(zero_, two_).Evaluate(), std::min(0.0, 2.0));
  EXPECT_DOUBLE_EQ(min(zero_, zero_).Evaluate(), std::min(0.0, 0.0));
  EXPECT_DOUBLE_EQ(min(zero_, neg_one_).Evaluate(), std::min(0.0, -1.0));
  EXPECT_DOUBLE_EQ(min(zero_, neg_pi_).Evaluate(), std::min(0.0, -3.141592));

  EXPECT_DOUBLE_EQ(min(neg_one_, pi_).Evaluate(), std::min(-1.0, 3.141592));
  EXPECT_DOUBLE_EQ(min(neg_one_, one_).Evaluate(), std::min(-1.0, 1.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, two_).Evaluate(), std::min(-1.0, 2.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, zero_).Evaluate(), std::min(-1.0, 0.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, neg_one_).Evaluate(), std::min(-1.0, -1.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, neg_pi_).Evaluate(),
                   std::min(-1.0, -3.141592));

  EXPECT_DOUBLE_EQ(min(neg_pi_, pi_).Evaluate(), std::min(-3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(min(neg_pi_, one_).Evaluate(), std::min(-3.141592, 1.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, two_).Evaluate(), std::min(-3.141592, 2.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, zero_).Evaluate(), std::min(-3.141592, 0.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, neg_one_).Evaluate(),
                   std::min(-3.141592, -1.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, neg_pi_).Evaluate(),
                   std::min(-3.141592, -3.141592));

  const Expression e{min(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::min(2 * 3.2 * 3.141592, std::sin(2) + std::sin(3.2)));
}

TEST_F(SymbolicExpressionTest, Max1) {
  // max(E, E) -> E
  EXPECT_TRUE(max(x_plus_y_, x_plus_y_).EqualTo(x_plus_y_));
}

TEST_F(SymbolicExpressionTest, Max2) {
  EXPECT_DOUBLE_EQ(max(pi_, pi_).Evaluate(), std::max(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(max(pi_, one_).Evaluate(), std::max(3.141592, 1.0));
  EXPECT_DOUBLE_EQ(max(pi_, two_).Evaluate(), std::max(3.141592, 2.0));
  EXPECT_DOUBLE_EQ(max(pi_, zero_).Evaluate(), std::max(3.141592, 0.0));
  EXPECT_DOUBLE_EQ(max(pi_, neg_one_).Evaluate(), std::max(3.141592, -1.0));
  EXPECT_DOUBLE_EQ(max(pi_, neg_pi_).Evaluate(), std::max(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(max(one_, pi_).Evaluate(), std::max(1.0, 3.141592));
  EXPECT_DOUBLE_EQ(max(one_, one_).Evaluate(), std::max(1.0, 1.0));
  EXPECT_DOUBLE_EQ(max(one_, two_).Evaluate(), std::max(1.0, 2.0));
  EXPECT_DOUBLE_EQ(max(one_, zero_).Evaluate(), std::max(1.0, 0.0));
  EXPECT_DOUBLE_EQ(max(one_, neg_one_).Evaluate(), std::max(1.0, -1.0));
  EXPECT_DOUBLE_EQ(max(one_, neg_pi_).Evaluate(), std::max(1.0, -3.141592));

  EXPECT_DOUBLE_EQ(max(two_, pi_).Evaluate(), std::max(2.0, 3.141592));
  EXPECT_DOUBLE_EQ(max(two_, one_).Evaluate(), std::max(2.0, 1.0));
  EXPECT_DOUBLE_EQ(max(two_, two_).Evaluate(), std::max(2.0, 2.0));
  EXPECT_DOUBLE_EQ(max(two_, zero_).Evaluate(), std::max(2.0, 0.0));
  EXPECT_DOUBLE_EQ(max(two_, neg_one_).Evaluate(), std::max(2.0, -1.0));
  EXPECT_DOUBLE_EQ(max(two_, neg_pi_).Evaluate(), std::max(2.0, -3.141592));

  EXPECT_DOUBLE_EQ(max(zero_, pi_).Evaluate(), std::max(0.0, 3.141592));
  EXPECT_DOUBLE_EQ(max(zero_, one_).Evaluate(), std::max(0.0, 1.0));
  EXPECT_DOUBLE_EQ(max(zero_, two_).Evaluate(), std::max(0.0, 2.0));
  EXPECT_DOUBLE_EQ(max(zero_, zero_).Evaluate(), std::max(0.0, 0.0));
  EXPECT_DOUBLE_EQ(max(zero_, neg_one_).Evaluate(), std::max(0.0, -1.0));
  EXPECT_DOUBLE_EQ(max(zero_, neg_pi_).Evaluate(), std::max(0.0, -3.141592));

  EXPECT_DOUBLE_EQ(max(neg_one_, pi_).Evaluate(), std::max(-1.0, 3.141592));
  EXPECT_DOUBLE_EQ(max(neg_one_, one_).Evaluate(), std::max(-1.0, 1.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, two_).Evaluate(), std::max(-1.0, 2.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, zero_).Evaluate(), std::max(-1.0, 0.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, neg_one_).Evaluate(), std::max(-1.0, -1.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, neg_pi_).Evaluate(),
                   std::max(-1.0, -3.141592));

  EXPECT_DOUBLE_EQ(max(neg_pi_, pi_).Evaluate(), std::max(-3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(max(neg_pi_, one_).Evaluate(), std::max(-3.141592, 1.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, two_).Evaluate(), std::max(-3.141592, 2.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, zero_).Evaluate(), std::max(-3.141592, 0.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, neg_one_).Evaluate(),
                   std::max(-3.141592, -1.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, neg_pi_).Evaluate(),
                   std::max(-3.141592, -3.141592));

  const Expression e{max(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::max(2 * 3.2 * 3.141592, std::sin(2) + std::sin(3.2)));
}

TEST_F(SymbolicExpressionTest, GetVariables) {
  const Variables vars1{(x_ + y_ * log(x_ + y_)).GetVariables()};
  EXPECT_TRUE(vars1.include(var_x_));
  EXPECT_TRUE(vars1.include(var_y_));
  EXPECT_FALSE(vars1.include(var_z_));
  EXPECT_EQ(vars1.size(), 2u);

  const Variables vars2{
      (x_ * x_ * z_ - y_ * abs(x_) * log(x_ + y_) + cosh(x_) + cosh(y_))
          .GetVariables()};
  EXPECT_TRUE(vars2.include(var_x_));
  EXPECT_TRUE(vars2.include(var_y_));
  EXPECT_TRUE(vars2.include(var_z_));
  EXPECT_EQ(vars2.size(), 3u);
}

TEST_F(SymbolicExpressionTest, Swap) {
  Expression e1{sin(x_ + y_ * z_)};
  Expression e2{(x_ * x_ + pow(y_, 2) * z_)};
  const Expression e1_copy{e1};
  const Expression e2_copy{e2};

  // Before Swap.
  EXPECT_TRUE(e1.EqualTo(e1_copy));
  EXPECT_TRUE(e2.EqualTo(e2_copy));

  swap(e1, e2);

  // After Swap.
  EXPECT_TRUE(e1.EqualTo(e2_copy));
  EXPECT_TRUE(e2.EqualTo(e1_copy));
}

TEST_F(SymbolicExpressionTest, ToString) {
  const Expression e1{sin(x_ + y_ * z_)};
  const Expression e2{cos(x_ * x_ + pow(y_, 2) * z_)};

  EXPECT_EQ(e1.to_string(), "sin((x + (y * z)))");
  EXPECT_EQ(e2.to_string(), "cos(((x * x) + (pow(y, 2) * z)))");
}

class SymbolicExpressionMatrixTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{3.141592};
  const Expression neg_pi_{-3.141592};
  const Expression e_{2.718};

  Eigen::Matrix<Expression, 3, 2> A_;
  Eigen::Matrix<Expression, 2, 3> B_;
  Eigen::Matrix<Expression, 3, 2> C_;

  void SetUp() {
    // clang-format off
    A_ << x_, one_,       //  [x  1]
          y_, neg_one_,   //  [y -1]
          z_, pi_;        //  [z  3.141592]

    B_ << x_, y_,  z_,    //  [x     y        z]
          e_, pi_, two_;  //  [2.718 3.141592 2]

    C_ << z_, two_,       //  [z  2]
          x_, e_,         //  [x -2.718]
          y_, pi_;        //  [y  3.141592]
    // clang-format on
  }
};

TEST_F(SymbolicExpressionMatrixTest, EigenAdd) {
  auto const M = A_ + A_;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ + x_), (one_ + one_),
                (y_ + y_), (neg_one_ + neg_one_),
                (z_ + z_), (pi_ + pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub1) {
  auto const M = A_ - A_;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub2) {
  auto const M = A_ - C_;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ - z_), (one_ - two_),
                (y_ - x_), (neg_one_ - e_),
                (z_ - y_), (pi_ - pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul1) {
  auto const M = A_ * B_;
  Eigen::Matrix<Expression, 3, 3> M_expected;
  // clang-format off
  M_expected <<
    (x_ * x_ + e_),       (x_ * y_ + pi_),       (x_ * z_ + two_),
    (y_ * x_ + -e_),      (y_ * y_ + - pi_),     (y_ * z_ + - two_),
    (z_ * x_ + pi_ * e_), (z_ * y_ + pi_ * pi_), (z_ * z_ + pi_ * two_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul2) {
  auto const M = B_ * A_;
  Eigen::Matrix<Expression, 2, 2> M_expected;
  // clang-format off
  M_expected <<
    (x_ * x_ + (y_ * y_ + z_ * z_)),      (x_ + (-y_ + z_ * pi_)),
    (e_ * x_ + (pi_ * y_ + two_ * z_)), (e_ * one_ + pi_ * - one_ + two_ * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul3) {
  auto const M = 2 * A_;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (2 * x_), (2 * one_),
                (2 * y_), (2 * neg_one_),
                (2 * z_), (2 * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul4) {
  auto const M = A_ * 2;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ * 2), (one_ * 2),
                (y_ * 2), (neg_one_ * 2),
                (z_ * 2), (pi_ * 2);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenDiv) {
  auto const M = A_ / 2;
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ / 2), (one_ / 2),
                (y_ / 2), (neg_one_ / 2),
                (z_ / 2), (pi_ / 2);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
