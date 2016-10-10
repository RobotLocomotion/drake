#include "drake/common/symbolic_expression.h"

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/primitives/gain-inl.h"
#include "drake/systems/framework/system_input.h"

using std::domain_error;
using std::equal_to;
using std::make_unique;
using std::runtime_error;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

GTEST_TEST(ExpressionTest, Variable) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  EXPECT_EQ(x.to_string(), var_x.get_name());
  EXPECT_EQ(y.to_string(), var_y.get_name());
  EXPECT_EQ(z.to_string(), var_z.get_name());
  EXPECT_TRUE(x.EqualTo(x));
  EXPECT_FALSE(x.EqualTo(y));
  EXPECT_FALSE(x.EqualTo(z));
  EXPECT_FALSE(y.EqualTo(x));
  EXPECT_TRUE(y.EqualTo(y));
  EXPECT_FALSE(y.EqualTo(z));
  EXPECT_FALSE(z.EqualTo(x));
  EXPECT_FALSE(z.EqualTo(y));
  EXPECT_TRUE(z.EqualTo(z));
}

GTEST_TEST(ExpressionTest, Constant) {
  const Expression c1{0.0};
  const Expression c2{1.0};
  const Expression c3{3.14159};
  const Expression c4{-2.718};
  EXPECT_EQ(c1.Evaluate(), 0);
  EXPECT_EQ(c2.Evaluate(), 1);
  EXPECT_EQ(c3.Evaluate(), 3.14159);
  EXPECT_EQ(c4.Evaluate(), -2.718);
  EXPECT_THROW(Expression{NAN}, runtime_error);
}

GTEST_TEST(ExpressionTest, StaticConstant) {
  EXPECT_DOUBLE_EQ(Expression::Zero().Evaluate(), 0.0);
  EXPECT_DOUBLE_EQ(Expression::One().Evaluate(), 1.0);
  EXPECT_NEAR(Expression::Pi().Evaluate(), 3.141592, 0.000001);
  EXPECT_NEAR(Expression::E().Evaluate(), 2.718281828, 0.000000001);
}

GTEST_TEST(ExpressionTest, HASH1) {
  const Variable var_x{"x"};
  Expression x{var_x};
  const Expression x_prime(x);
  EXPECT_EQ(x.get_hash(), x_prime.get_hash());
  x++;
  EXPECT_NE(x.get_hash(), x_prime.get_hash());
}

GTEST_TEST(ExpressionTest, HASH_BINARY) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression x_plus_y{x + y};

  const Expression e1{x_plus_y + x_plus_y};
  const Expression e2{x_plus_y - x_plus_y};
  const Expression e3{x_plus_y * x_plus_y};
  const Expression e4{x_plus_y / x_plus_y};
  const Expression e5{pow(x_plus_y, x_plus_y)};
  const Expression e6{atan2(x_plus_y, x_plus_y)};

  // e1, ..., e6 share the same sub-expressions, but their hash values should be
  // distinct.
  unordered_set<size_t> hash_set;
  for (auto const& e : {e1, e2, e3, e4, e5, e6}) {
    hash_set.insert(e.get_hash());
  }
  EXPECT_EQ(hash_set.size(), 6u);
}

GTEST_TEST(ExpressionTest, HASH_UNARY) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression x_plus_y{x + y};

  const Expression e0{log(x_plus_y)};
  const Expression e1{abs(x_plus_y)};
  const Expression e2{exp(x_plus_y)};
  const Expression e3{sqrt(x_plus_y)};
  const Expression e4{sin(x_plus_y)};
  const Expression e5{cos(x_plus_y)};
  const Expression e6{tan(x_plus_y)};
  const Expression e7{asin(x_plus_y)};
  const Expression e8{acos(x_plus_y)};
  const Expression e9{atan(x_plus_y)};
  const Expression e10{sinh(x_plus_y)};
  const Expression e11{cosh(x_plus_y)};
  const Expression e12{tanh(x_plus_y)};

  // e0, ..., e12 share the same sub-expression, but their hash values should be
  // distinct.
  unordered_set<size_t> hash_set;
  for (auto const& e :
       {e0, e1, e2, e3, e4, e5, e6, e7, e8, e9, e10, e11, e12}) {
    hash_set.insert(e.get_hash());
  }
  EXPECT_EQ(hash_set.size(), 13u);
}

GTEST_TEST(ExpressionTest, UNARY_MINUS) {
  const Expression c1{3.14159};
  EXPECT_FALSE(c1.EqualTo(-c1));
  EXPECT_DOUBLE_EQ(c1.Evaluate(), -(-c1).Evaluate());
  // c1 and -(-c1) are structurally different
  EXPECT_FALSE(c1.EqualTo(-(-c1)));
  // but their evaluations should be the same.
  EXPECT_DOUBLE_EQ(c1.Evaluate(), (-(-c1)).Evaluate());
}

GTEST_TEST(ExpressionTest, ADD1) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression zero{0.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  EXPECT_TRUE((c1 + zero).EqualTo(c1));
  EXPECT_EQ((c1 + zero).to_string(), c1.to_string());
  EXPECT_TRUE((zero + c1).EqualTo(c1));
  EXPECT_EQ((zero + c1).to_string(), c1.to_string());
  EXPECT_TRUE((0.0 + c1).EqualTo(c1));
  EXPECT_EQ((0.0 + c1).to_string(), c1.to_string());
  EXPECT_TRUE((c1 + 0.0).EqualTo(c1));
  EXPECT_EQ((c1 + 0.0).to_string(), c1.to_string());
  EXPECT_TRUE(equal_to<Expression>{}(c1 + c2, Expression{3.14159 + -2.718}));
  EXPECT_EQ((c1 + c2).to_string(), Expression{3.14159 + -2.718}.to_string());
  EXPECT_TRUE(equal_to<Expression>{}(c1 + x, 3.14159 + x));
  EXPECT_EQ((c1 + x).to_string(), (3.14159 + x).to_string());
  EXPECT_TRUE(equal_to<Expression>{}(x + c1, x + 3.14159));
  EXPECT_EQ((x + c1).to_string(), (x + 3.14159).to_string());
}

GTEST_TEST(ExpressionTest, ADD2) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  const Expression zero{0.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  Expression e1{x + y};
  Expression e2{e1 + e1};
  EXPECT_EQ(e1.to_string(), "(x + y)");
  EXPECT_EQ(e2.to_string(), "((x + y) + (x + y))");
  e1 += z;
  EXPECT_EQ(e1.to_string(), "((x + y) + z)");
  EXPECT_EQ(e2.to_string(), "((x + y) + (x + y))");  // e2 doesn't change.
}

GTEST_TEST(ExpressionTest, INC1) {
  // Prefix increment
  const Variable var_x{"x"};
  Expression x{var_x};
  Expression x_prime{var_x};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_TRUE(x++.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime++));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

GTEST_TEST(ExpressionTest, INC2) {
  // Postfix increment
  const Variable var_x{"x"};
  Expression x{var_x};
  Expression x_prime{var_x};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_FALSE((++x).EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_TRUE(x.EqualTo(++x_prime));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

GTEST_TEST(ExpressionTest, INC3) {
  // Pre/Post increments
  Expression c1{3.1415};
  EXPECT_DOUBLE_EQ((c1++).Evaluate(), 3.1415);
  EXPECT_DOUBLE_EQ(c1.Evaluate(), 3.1415 + 1.0);

  Expression c2{3.1415};
  EXPECT_DOUBLE_EQ((++c2).Evaluate(), 3.1415 + 1.0);
  EXPECT_DOUBLE_EQ(c2.Evaluate(), 3.1415 + 1.0);
}

GTEST_TEST(ExpressionTest, SUB1) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression zero{0.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  EXPECT_EQ((c1 - zero).to_string(), c1.to_string());
  EXPECT_EQ((zero - c1).to_string(), Expression{-3.14159}.to_string());
  EXPECT_EQ((0.0 - c1).to_string(), Expression{-3.14159}.to_string());
  EXPECT_EQ((0.0 - c1).to_string(), (-1 * c1).to_string());
  EXPECT_EQ((c1 - 0.0).to_string(), c1.to_string());
  EXPECT_EQ((c1 - c2).to_string(), Expression{3.14159 - -2.718}.to_string());
  EXPECT_EQ((c1 - x).to_string(), (3.14159 - x).to_string());
  EXPECT_EQ((x - c1).to_string(), (x - 3.14159).to_string());
}

GTEST_TEST(ExpressionTest, SUB2) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  const Expression zero{0.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  Expression e1{x - y};
  Expression e2{x - z};
  Expression e3{e1 - e2};
  EXPECT_EQ(e1.to_string(), "(x - y)");
  EXPECT_EQ(e2.to_string(), "(x - z)");
  EXPECT_EQ(e3.to_string(), "((x - y) - (x - z))");
  e1 -= z;
  EXPECT_EQ(e1.to_string(), "((x - y) - z)");
  EXPECT_EQ(e3.to_string(), "((x - y) - (x - z))");  // e3 doesn't change.
}

GTEST_TEST(ExpressionTest, SUB3) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};

  Expression e1{x - y};
  Expression e2{x - y};
  Expression e3{e1 - e2};
  EXPECT_EQ(e1.to_string(), "(x - y)");
  EXPECT_EQ(e2.to_string(), "(x - y)");
  EXPECT_EQ(e3.to_string(), "0");  // simplified
}

GTEST_TEST(ExpressionTest, DEC1) {
  // Postfix decrement.
  const Variable var_x{"x"};
  Expression x{var_x};
  Expression x_prime{var_x};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_TRUE(x--.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime--));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

GTEST_TEST(ExpressionTest, DEC2) {
  // Prefix decrement.
  const Variable var_x{"x"};
  Expression x{var_x};
  Expression x_prime{var_x};

  EXPECT_TRUE(x.EqualTo(x_prime));
  EXPECT_FALSE((--x).EqualTo(x_prime));
  EXPECT_FALSE(x.EqualTo(x_prime));
  EXPECT_TRUE(x.EqualTo(--x_prime));
  EXPECT_TRUE(x.EqualTo(x_prime));
}

GTEST_TEST(ExpressionTest, DEC3) {
  // Pre/Postfix decrements.
  Expression c1{3.1415};
  EXPECT_DOUBLE_EQ((c1--).Evaluate(), 3.1415);
  EXPECT_DOUBLE_EQ(c1.Evaluate(), 3.1415 - 1.0);

  Expression c2{3.1415};
  EXPECT_DOUBLE_EQ((--c2).Evaluate(), 3.1415 - 1.0);
  EXPECT_DOUBLE_EQ(c2.Evaluate(), 3.1415 - 1.0);
}

GTEST_TEST(ExpressionTest, MUL1) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression zero{0.0};
  const Expression one{1.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  EXPECT_EQ((c1 * zero).to_string(), zero.to_string());
  EXPECT_EQ((zero * c1).to_string(), zero.to_string());
  EXPECT_EQ((c1 * 0.0).to_string(), zero.to_string());
  EXPECT_EQ((0.0 * c1).to_string(), zero.to_string());

  EXPECT_EQ((c1 * one).to_string(), c1.to_string());
  EXPECT_EQ((one * c1).to_string(), c1.to_string());
  EXPECT_EQ((1.0 * c1).to_string(), c1.to_string());
  EXPECT_EQ((c1 * 1.0).to_string(), c1.to_string());

  EXPECT_EQ((c1 * c2).to_string(), Expression{3.14159 * -2.718}.to_string());
  EXPECT_EQ((c1 * x).to_string(), (3.14159 * x).to_string());
  EXPECT_EQ((x * c1).to_string(), (x * 3.14159).to_string());
}

GTEST_TEST(ExpressionTest, MUL2) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  const Expression zero{0.0};
  const Expression one{1.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  Expression e1{x * y};
  Expression e2{e1 * e1};
  EXPECT_EQ(e1.to_string(), "(x * y)");
  EXPECT_EQ(e2.to_string(), "((x * y) * (x * y))");
  e1 *= z;
  EXPECT_EQ(e1.to_string(), "((x * y) * z)");
  EXPECT_EQ(e2.to_string(), "((x * y) * (x * y))");  // e2 doesn't change.
}

GTEST_TEST(ExpressionTest, DIV1) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression zero{0.0};
  const Expression one{1.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  EXPECT_THROW(c1 / zero, runtime_error);
  EXPECT_EQ((zero / c1).to_string(), zero.to_string());
  EXPECT_THROW(c1 / 0.0, runtime_error);
  EXPECT_EQ((0.0 / c1).to_string(), zero.to_string());

  EXPECT_EQ((c1 / one).to_string(), c1.to_string());
  EXPECT_EQ((c1 / 1.0).to_string(), c1.to_string());

  EXPECT_EQ((c1 / c2).to_string(), Expression{3.14159 / -2.718}.to_string());
  EXPECT_EQ((c1 / x).to_string(), (3.14159 / x).to_string());
  EXPECT_EQ((x / c1).to_string(), (x / 3.14159).to_string());
}

GTEST_TEST(ExpressionTest, DIV2) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  const Expression zero{0.0};
  const Expression one{1.0};
  const Expression c1{3.14159};
  const Expression c2{-2.718};

  Expression e1{x / y};
  Expression e2{x / z};
  Expression e3{e1 / e2};
  EXPECT_EQ(e1.to_string(), "(x / y)");
  EXPECT_EQ(e2.to_string(), "(x / z)");
  EXPECT_EQ(e3.to_string(), "((x / y) / (x / z))");
  e1 /= z;
  EXPECT_EQ(e1.to_string(), "((x / y) / z)");
  EXPECT_EQ(e3.to_string(), "((x / y) / (x / z))");  // e2 doesn't change.
}

GTEST_TEST(ExpressionTest, DIV3) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};

  Expression e1{x / y};
  Expression e2{x / y};
  Expression e3{e1 / e2};
  EXPECT_EQ(e1.to_string(), "(x / y)");
  EXPECT_EQ(e2.to_string(), "(x / y)");
  EXPECT_EQ(e3.to_string(), "1");  // simplified
}

// This test checks whether symbolic::Expression is compatible with
// std::unordered_set.
GTEST_TEST(ExpressionTest, compatible_with_unordered_set) {
  unordered_set<Expression> uset;
  uset.emplace(Expression{Variable{"a"}});
  uset.emplace(Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::unordered_map.
GTEST_TEST(ExpressionTest, compatible_with_unordered_map) {
  unordered_map<Expression, Expression> umap;
  umap.emplace(Expression{Variable{"a"}}, Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::vector.
GTEST_TEST(ExpressionTest, compatible_with_vector) {
  vector<Expression> vec;
  vec.push_back(Expression{123.0});
}

GTEST_TEST(ExpressionTest, log) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(log(pi).Evaluate(), std::log(3.141592));
  EXPECT_DOUBLE_EQ(log(one).Evaluate(), std::log(1.0));
  EXPECT_DOUBLE_EQ(log(zero).Evaluate(), std::log(0.0));
  EXPECT_THROW(log(neg_one).Evaluate(), domain_error);
  EXPECT_THROW(log(neg_pi).Evaluate(), domain_error);

  const Expression e{log(x * y * pi) + log(x) + log(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::log(2 * 3.2 * 3.141592) + std::log(2) + std::log(3.2));
}

GTEST_TEST(ExpressionTest, abs) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(abs(pi).Evaluate(), std::fabs(3.141592));
  EXPECT_DOUBLE_EQ(abs(one).Evaluate(), std::fabs(1.0));
  EXPECT_DOUBLE_EQ(abs(zero).Evaluate(), std::fabs(0.0));
  EXPECT_DOUBLE_EQ(abs(neg_one).Evaluate(), std::fabs(-1.0));
  EXPECT_DOUBLE_EQ(abs(neg_pi).Evaluate(), std::fabs(-3.141592));

  const Expression e{abs(x * y * pi) + abs(x) + abs(y)};
  const Environment env{{var_x, -2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::fabs(-2 * 3.2 * 3.141592) +
                                        std::fabs(-2.0) + std::fabs(3.2));
}

GTEST_TEST(ExpressionTest, exp) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};
  EXPECT_DOUBLE_EQ(exp(pi).Evaluate(), std::exp(3.141592));
  EXPECT_DOUBLE_EQ(exp(one).Evaluate(), std::exp(1));
  EXPECT_DOUBLE_EQ(exp(zero).Evaluate(), std::exp(0));
  EXPECT_DOUBLE_EQ(exp(neg_one).Evaluate(), std::exp(-1));
  EXPECT_DOUBLE_EQ(exp(neg_pi).Evaluate(), std::exp(-3.141592));

  const Expression e{exp(x * y * pi) + exp(x) + exp(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::exp(2 * 3.2 * 3.141592) +
                                        std::exp(2.0) + std::exp(3.2));
}

GTEST_TEST(ExpressionTest, sqrt) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};
  EXPECT_DOUBLE_EQ(sqrt(pi).Evaluate(), std::sqrt(3.141592));
  EXPECT_DOUBLE_EQ(sqrt(one).Evaluate(), std::sqrt(1.0));
  EXPECT_DOUBLE_EQ(sqrt(zero).Evaluate(), std::sqrt(0.0));
  EXPECT_THROW(sqrt(neg_one).Evaluate(), domain_error);
  EXPECT_THROW(sqrt(neg_pi).Evaluate(), domain_error);

  const Expression e{sqrt(x * y * pi) + sqrt(x) + sqrt(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::sqrt(2 * 3.2 * 3.141592) +
                                        std::sqrt(2.0) + std::sqrt(3.2));
}

GTEST_TEST(ExpressionTest, pow) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};
  EXPECT_DOUBLE_EQ(pow(pi, pi).Evaluate(), std::pow(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(pow(pi, one).Evaluate(), std::pow(3.141592, 1));
  EXPECT_DOUBLE_EQ(pow(pi, two).Evaluate(), std::pow(3.141592, 2));
  EXPECT_DOUBLE_EQ(pow(pi, zero).Evaluate(), std::pow(3.141592, 0));
  EXPECT_DOUBLE_EQ(pow(pi, neg_one).Evaluate(), std::pow(3.141592, -1));
  EXPECT_DOUBLE_EQ(pow(pi, neg_pi).Evaluate(), std::pow(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(pow(one, pi).Evaluate(), std::pow(1, 3.141592));
  EXPECT_DOUBLE_EQ(pow(one, one).Evaluate(), std::pow(1, 1));
  EXPECT_DOUBLE_EQ(pow(one, two).Evaluate(), std::pow(1, 2));
  EXPECT_DOUBLE_EQ(pow(one, zero).Evaluate(), std::pow(1, 0));
  EXPECT_DOUBLE_EQ(pow(one, neg_one).Evaluate(), std::pow(1, -1));
  EXPECT_DOUBLE_EQ(pow(one, neg_pi).Evaluate(), std::pow(1, -3.141592));

  EXPECT_DOUBLE_EQ(pow(two, pi).Evaluate(), std::pow(2, 3.141592));
  EXPECT_DOUBLE_EQ(pow(two, one).Evaluate(), std::pow(2, 1));
  EXPECT_DOUBLE_EQ(pow(two, two).Evaluate(), std::pow(2, 2));
  EXPECT_DOUBLE_EQ(pow(two, zero).Evaluate(), std::pow(2, 0));
  EXPECT_DOUBLE_EQ(pow(two, neg_one).Evaluate(), std::pow(2, -1));
  EXPECT_DOUBLE_EQ(pow(two, neg_pi).Evaluate(), std::pow(2, -3.141592));

  EXPECT_DOUBLE_EQ(pow(zero, pi).Evaluate(), std::pow(0, 3.141592));
  EXPECT_DOUBLE_EQ(pow(zero, one).Evaluate(), std::pow(0, 1));
  EXPECT_DOUBLE_EQ(pow(zero, two).Evaluate(), std::pow(0, 2));
  EXPECT_DOUBLE_EQ(pow(zero, zero).Evaluate(), std::pow(0, 0));
  EXPECT_DOUBLE_EQ(pow(zero, neg_one).Evaluate(), std::pow(0, -1));
  EXPECT_DOUBLE_EQ(pow(zero, neg_pi).Evaluate(), std::pow(0, -3.141592));

  EXPECT_THROW(pow(neg_one, pi).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_one, one).Evaluate(), std::pow(-1, 1));
  EXPECT_DOUBLE_EQ(pow(neg_one, two).Evaluate(), std::pow(-1, 2));
  EXPECT_DOUBLE_EQ(pow(neg_one, zero).Evaluate(), std::pow(-1, 0));
  EXPECT_DOUBLE_EQ(pow(neg_one, neg_one).Evaluate(), std::pow(-1, -1));
  EXPECT_THROW(pow(neg_one, neg_pi).Evaluate(), domain_error);

  EXPECT_THROW(pow(neg_pi, pi).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_pi, one).Evaluate(), std::pow(-3.141592, 1));
  EXPECT_DOUBLE_EQ(pow(neg_pi, two).Evaluate(), std::pow(-3.141592, 2));
  EXPECT_DOUBLE_EQ(pow(neg_pi, zero).Evaluate(), std::pow(-3.141592, 0));
  EXPECT_DOUBLE_EQ(pow(neg_pi, neg_one).Evaluate(), std::pow(-3.141592, -1));
  EXPECT_THROW(pow(neg_pi, neg_pi).Evaluate(), domain_error);

  const Expression e{pow(x * y * pi, x + y + pi)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::pow(2 * 3.2 * 3.141592, 2 + 3.2 + 3.141592));
}

GTEST_TEST(ExpressionTest, sin) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(sin(pi).Evaluate(), std::sin(3.141592));
  EXPECT_DOUBLE_EQ(sin(one).Evaluate(), std::sin(1));
  EXPECT_DOUBLE_EQ(sin(two).Evaluate(), std::sin(2));
  EXPECT_DOUBLE_EQ(sin(zero).Evaluate(), std::sin(0));
  EXPECT_DOUBLE_EQ(sin(neg_one).Evaluate(), std::sin(-1));
  EXPECT_DOUBLE_EQ(sin(neg_pi).Evaluate(), std::sin(-3.141592));

  const Expression e{sin(x * y * pi) + sin(x) + sin(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::sin(2 * 3.2 * 3.141592) + std::sin(2) + std::sin(3.2));
}

GTEST_TEST(ExpressionTest, cos) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(cos(pi).Evaluate(), std::cos(3.141592));
  EXPECT_DOUBLE_EQ(cos(one).Evaluate(), std::cos(1));
  EXPECT_DOUBLE_EQ(cos(two).Evaluate(), std::cos(2));
  EXPECT_DOUBLE_EQ(cos(zero).Evaluate(), std::cos(0));
  EXPECT_DOUBLE_EQ(cos(neg_one).Evaluate(), std::cos(-1));
  EXPECT_DOUBLE_EQ(cos(neg_pi).Evaluate(), std::cos(-3.141592));
  const Expression e{cos(x * y * pi) + cos(x) + cos(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::cos(2 * 3.2 * 3.141592) + std::cos(2) + std::cos(3.2));
}

GTEST_TEST(ExpressionTest, tan) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(tan(pi).Evaluate(), std::tan(3.141592));
  EXPECT_DOUBLE_EQ(tan(one).Evaluate(), std::tan(1));
  EXPECT_DOUBLE_EQ(tan(two).Evaluate(), std::tan(2));
  EXPECT_DOUBLE_EQ(tan(zero).Evaluate(), std::tan(0));
  EXPECT_DOUBLE_EQ(tan(neg_one).Evaluate(), std::tan(-1));
  EXPECT_DOUBLE_EQ(tan(neg_pi).Evaluate(), std::tan(-3.141592));

  const Expression e{tan(x * y * pi) + tan(x) + tan(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::tan(2 * 3.2 * 3.141592) + std::tan(2) + std::tan(3.2));
}

GTEST_TEST(ExpressionTest, asin) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_THROW(asin(pi).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(asin(one).Evaluate(), std::asin(1));
  EXPECT_THROW(asin(two).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(asin(zero).Evaluate(), std::asin(0));
  EXPECT_DOUBLE_EQ(asin(neg_one).Evaluate(), std::asin(-1));
  EXPECT_THROW(asin(neg_pi).Evaluate(), domain_error);

  const Expression e{asin(x * y * pi) + asin(x) + asin(y)};
  const Environment env{{var_x, 0.2}, {var_y, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::asin(0.2 * 0.3 * 3.141592) +
                                        std::asin(0.2) + std::asin(0.3));
}

GTEST_TEST(ExpressionTest, acos) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_THROW(acos(pi).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(acos(one).Evaluate(), std::acos(1));
  EXPECT_THROW(acos(two).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(acos(zero).Evaluate(), std::acos(0));
  EXPECT_DOUBLE_EQ(acos(neg_one).Evaluate(), std::acos(-1));
  EXPECT_THROW(acos(neg_pi).Evaluate(), domain_error);

  const Expression e{acos(x * y * pi) + acos(x) + acos(y)};
  const Environment env{{var_x, 0.2}, {var_y, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::acos(0.2 * 0.3 * 3.141592) +
                                        std::acos(0.2) + std::acos(0.3));
}

GTEST_TEST(ExpressionTest, atan) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(atan(pi).Evaluate(), std::atan(3.141592));
  EXPECT_DOUBLE_EQ(atan(one).Evaluate(), std::atan(1));
  EXPECT_DOUBLE_EQ(atan(two).Evaluate(), std::atan(2));
  EXPECT_DOUBLE_EQ(atan(zero).Evaluate(), std::atan(0));
  EXPECT_DOUBLE_EQ(atan(neg_one).Evaluate(), std::atan(-1));
  EXPECT_DOUBLE_EQ(atan(neg_pi).Evaluate(), std::atan(-3.141592));

  const Expression e{atan(x * y * pi) + atan(x) + atan(y)};
  const Environment env{{var_x, 0.2}, {var_y, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::atan(0.2 * 0.3 * 3.141592) +
                                        std::atan(0.2) + std::atan(0.3));
}

GTEST_TEST(ExpressionTest, atan2) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};
  EXPECT_DOUBLE_EQ(atan2(pi, pi).Evaluate(), std::atan2(3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(pi, one).Evaluate(), std::atan2(3.141592, 1));
  EXPECT_DOUBLE_EQ(atan2(pi, two).Evaluate(), std::atan2(3.141592, 2));
  EXPECT_DOUBLE_EQ(atan2(pi, zero).Evaluate(), std::atan2(3.141592, 0));
  EXPECT_DOUBLE_EQ(atan2(pi, neg_one).Evaluate(), std::atan2(3.141592, -1));
  EXPECT_DOUBLE_EQ(atan2(pi, neg_pi).Evaluate(),
                   std::atan2(3.141592, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(one, pi).Evaluate(), std::atan2(1, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(one, one).Evaluate(), std::atan2(1, 1));
  EXPECT_DOUBLE_EQ(atan2(one, two).Evaluate(), std::atan2(1, 2));
  EXPECT_DOUBLE_EQ(atan2(one, zero).Evaluate(), std::atan2(1, 0));
  EXPECT_DOUBLE_EQ(atan2(one, neg_one).Evaluate(), std::atan2(1, -1));
  EXPECT_DOUBLE_EQ(atan2(one, neg_pi).Evaluate(), std::atan2(1, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(two, pi).Evaluate(), std::atan2(2, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(two, one).Evaluate(), std::atan2(2, 1));
  EXPECT_DOUBLE_EQ(atan2(two, two).Evaluate(), std::atan2(2, 2));
  EXPECT_DOUBLE_EQ(atan2(two, zero).Evaluate(), std::atan2(2, 0));
  EXPECT_DOUBLE_EQ(atan2(two, neg_one).Evaluate(), std::atan2(2, -1));
  EXPECT_DOUBLE_EQ(atan2(two, neg_pi).Evaluate(), std::atan2(2, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(zero, pi).Evaluate(), std::atan2(0, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(zero, one).Evaluate(), std::atan2(0, 1));
  EXPECT_DOUBLE_EQ(atan2(zero, two).Evaluate(), std::atan2(0, 2));
  EXPECT_DOUBLE_EQ(atan2(zero, zero).Evaluate(), std::atan2(0, 0));
  EXPECT_DOUBLE_EQ(atan2(zero, neg_one).Evaluate(), std::atan2(0, -1));
  EXPECT_DOUBLE_EQ(atan2(zero, neg_pi).Evaluate(), std::atan2(0, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(neg_one, pi).Evaluate(), std::atan2(-1, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(neg_one, one).Evaluate(), std::atan2(-1, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_one, two).Evaluate(), std::atan2(-1, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_one, zero).Evaluate(), std::atan2(-1, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_one, neg_one).Evaluate(), std::atan2(-1, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_one, neg_pi).Evaluate(),
                   std::atan2(-1, -3.141592));

  EXPECT_DOUBLE_EQ(atan2(neg_pi, pi).Evaluate(),
                   std::atan2(-3.141592, 3.141592));
  EXPECT_DOUBLE_EQ(atan2(neg_pi, one).Evaluate(), std::atan2(-3.141592, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi, two).Evaluate(), std::atan2(-3.141592, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_pi, zero).Evaluate(), std::atan2(-3.141592, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_pi, neg_one).Evaluate(),
                   std::atan2(-3.141592, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi, neg_pi).Evaluate(),
                   std::atan2(-3.141592, -3.141592));

  const Expression e{atan2(x * y * pi, sin(x) + sin(y))};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::atan2(2 * 3.2 * 3.141592, std::sin(2) + std::sin(3.2)));
}

GTEST_TEST(ExpressionTest, sinh) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(sinh(pi).Evaluate(), std::sinh(3.141592));
  EXPECT_DOUBLE_EQ(sinh(one).Evaluate(), std::sinh(1));
  EXPECT_DOUBLE_EQ(sinh(two).Evaluate(), std::sinh(2));
  EXPECT_DOUBLE_EQ(sinh(zero).Evaluate(), std::sinh(0));
  EXPECT_DOUBLE_EQ(sinh(neg_one).Evaluate(), std::sinh(-1));
  EXPECT_DOUBLE_EQ(sinh(neg_pi).Evaluate(), std::sinh(-3.141592));

  const Expression e{sinh(x * y * pi) + sinh(x) + sinh(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::sinh(2 * 3.2 * 3.141592) +
                                        std::sinh(2) + std::sinh(3.2));
}

GTEST_TEST(ExpressionTest, cosh) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(cosh(pi).Evaluate(), std::cosh(3.141592));
  EXPECT_DOUBLE_EQ(cosh(one).Evaluate(), std::cosh(1));
  EXPECT_DOUBLE_EQ(cosh(two).Evaluate(), std::cosh(2));
  EXPECT_DOUBLE_EQ(cosh(zero).Evaluate(), std::cosh(0));
  EXPECT_DOUBLE_EQ(cosh(neg_one).Evaluate(), std::cosh(-1));
  EXPECT_DOUBLE_EQ(cosh(neg_pi).Evaluate(), std::cosh(-3.141592));

  const Expression e{cosh(x * y * pi) + cosh(x) + cosh(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::cosh(2 * 3.2 * 3.141592) +
                                        std::cosh(2) + std::cosh(3.2));
}

GTEST_TEST(ExpressionTest, tanh) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  EXPECT_DOUBLE_EQ(tanh(pi).Evaluate(), std::tanh(3.141592));
  EXPECT_DOUBLE_EQ(tanh(one).Evaluate(), std::tanh(1));
  EXPECT_DOUBLE_EQ(tanh(two).Evaluate(), std::tanh(2));
  EXPECT_DOUBLE_EQ(tanh(zero).Evaluate(), std::tanh(0));
  EXPECT_DOUBLE_EQ(tanh(neg_one).Evaluate(), std::tanh(-1));
  EXPECT_DOUBLE_EQ(tanh(neg_pi).Evaluate(), std::tanh(-3.141592));

  const Expression e{tanh(x * y * pi) + tanh(x) + tanh(y)};
  const Environment env{{var_x, 2}, {var_y, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::tanh(2 * 3.2 * 3.141592) +
                                        std::tanh(2) + std::tanh(3.2));
}

GTEST_TEST(ExpressionTest, GetVariables) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};

  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};

  Variables const vars1 = (x + y * log(x + y)).GetVariables();
  EXPECT_TRUE(vars1.include(var_x));
  EXPECT_TRUE(vars1.include(var_y));
  EXPECT_FALSE(vars1.include(var_z));
  EXPECT_EQ(vars1.size(), 2u);

  Variables const vars2 =
      (x * x * z - y * abs(x) * log(x + y) + cosh(x) + cosh(y)).GetVariables();
  EXPECT_TRUE(vars2.include(var_x));
  EXPECT_TRUE(vars2.include(var_y));
  EXPECT_TRUE(vars2.include(var_z));
  EXPECT_EQ(vars2.size(), 3u);
}

GTEST_TEST(ExpressionTest, swap) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};
  const Expression pi{3.141592};
  const Expression one{1};
  const Expression two{2};
  const Expression zero{0};
  const Expression neg_one{-one};
  const Expression neg_pi{-pi};

  Expression e1 = sin(x + y * z);
  Expression e2 = cos(x * x + pow(y, 2) * z);
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

GTEST_TEST(ExpressionTest, output_operator) {
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};

  const Expression e1 = sin(x + y * z);
  const Expression e2 = cos(x * x + pow(y, 2) * z);

  EXPECT_EQ(e1.to_string(), "sin((x + (y * z)))");
  EXPECT_EQ(e2.to_string(), "cos(((x * x) + (pow(y, 2) * z)))");
}
}  // namespace
}  // namespace symbolic
}  // namespace drake

// TODO(soonho): The following test is disabled for now, because
// LeafSystem<T> requires a typename T to have the following operations:
//
//  - operator<
//  - operator>
//  - floor
//  - ceil
//  - casting to int64_t
#if 0
namespace drake {
namespace systems {
namespace {

using Eigen::Matrix;
using std::unique_ptr;
using std::move;

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
  std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class GainTest : public ::testing::Test {
 protected:
  void SetUp() override {
    gain_ = make_unique<Gain<symbolic::Expression>>(kGain_ /* gain */,
                                                    3 /* length */);
    context_ = gain_->CreateDefaultContext();
    output_ = gain_->AllocateOutput(*context_);
    input0_ = make_unique<BasicVector<symbolic::Expression>>(3 /* length */);
    input1_ = make_unique<BasicVector<symbolic::Expression>>(3 /* length */);
  }

  const symbolic::Expression kGain_{2.0};
  unique_ptr<System<symbolic::Expression>> gain_;
  unique_ptr<Context<symbolic::Expression>> context_;
  unique_ptr<SystemOutput<symbolic::Expression>> output_;
  unique_ptr<BasicVector<symbolic::Expression>> input0_;
  unique_ptr<BasicVector<symbolic::Expression>> input1_;
};

TEST_F(GainTest, VectorThroughGainSystem) {
  // Checks that the number of input ports in the Gain system and the Context
  // are consistent.
  EXPECT_EQ(1, gain_->get_num_input_ports());
  EXPECT_EQ(1, context_->get_num_input_ports());
  Matrix<symbolic::Expression, 3, 1> input_vector(
    drake::symbolic::Expression{1.0}, drake::symbolic::Expression{3.14},
    drake::symbolic::Expression{2.18});
  input0_->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context_->SetInputPort(0, MakeInput(move(input0_)));
  gain_->EvalOutput(*context_, output_.get());

  // Checks that the number of output ports in the Gain system and the
  // SystemOutput are consistent.
  EXPECT_EQ(1, output_->get_num_ports());
  EXPECT_EQ(1, gain_->get_num_output_ports());
  const BasicVector<symbolic::Expression>* output_vector =
    output_->get_vector_data(0);
  EXPECT_NE(nullptr, output_vector);
  Matrix<symbolic::Expression, 3, 1> expected = kGain_ * input_vector;
  EXPECT_EQ(expected, output_vector->get_value());
  EXPECT_EQ(expected(0).Evaluate(), kGain_.Evaluate() * 1.0);
  EXPECT_EQ(expected(1).Evaluate(), kGain_.Evaluate() * 3.14);
  EXPECT_EQ(expected(2).Evaluate(), kGain_.Evaluate() * 2.18);
}
#endif

// }  // namespace
// }  // namespace systems
// }  // namespace drake
