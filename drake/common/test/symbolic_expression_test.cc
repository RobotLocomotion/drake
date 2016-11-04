#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

using std::domain_error;
using std::equal_to;
using std::map;
using std::runtime_error;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

bool ExpEqual(const Expression& e1, const Expression& e2) {
  return e1.EqualTo(e2);
}

bool ExpNotEqual(const Expression& e1, const Expression& e2) {
  return !ExpEqual(e1, e2);
}

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

  const Expression c1_{-10.0};
  const Expression c2_{1.0};
  const Expression c3_{3.14159};
  const Expression c4_{-2.718};
};

TEST_F(SymbolicExpressionTest, LessConstant) {
  EXPECT_FALSE(c1_.Less(c1_));
  EXPECT_TRUE(c1_.Less(c2_));
  EXPECT_TRUE(c1_.Less(c3_));
  EXPECT_FALSE(c2_.Less(c1_));
  EXPECT_FALSE(c2_.Less(c2_));
  EXPECT_TRUE(c2_.Less(c3_));
  EXPECT_FALSE(c3_.Less(c1_));
  EXPECT_FALSE(c3_.Less(c2_));
  EXPECT_FALSE(c3_.Less(c3_));
}

TEST_F(SymbolicExpressionTest, LessVariable) {
  EXPECT_FALSE(x_.Less(x_));
  EXPECT_TRUE(x_.Less(y_));
  EXPECT_TRUE(x_.Less(z_));
  EXPECT_FALSE(y_.Less(x_));
  EXPECT_FALSE(y_.Less(y_));
  EXPECT_TRUE(y_.Less(z_));
  EXPECT_FALSE(z_.Less(x_));
  EXPECT_FALSE(z_.Less(y_));
  EXPECT_FALSE(z_.Less(z_));
}

TEST_F(SymbolicExpressionTest, LessNeg) {
  // defined in the ascending order.
  const Expression neg1{-c3_};
  const Expression neg2{-c1_};
  const Expression neg3{-x_};  // note: Constant kind < Variable kind

  EXPECT_FALSE(neg1.Less(neg1));
  EXPECT_TRUE(neg1.Less(neg2));
  EXPECT_TRUE(neg1.Less(neg3));
  EXPECT_FALSE(neg2.Less(neg1));
  EXPECT_FALSE(neg2.Less(neg2));
  EXPECT_TRUE(neg2.Less(neg3));
  EXPECT_FALSE(neg3.Less(neg1));
  EXPECT_FALSE(neg3.Less(neg2));
  EXPECT_FALSE(neg3.Less(neg3));
}

TEST_F(SymbolicExpressionTest, LessAdd) {
  const Expression add1{c1_ + x_ + y_};
  const Expression add2{c1_ + y_ + z_};
  const Expression add3{c3_ + x_ + y_};
  const Expression add4{c3_ + y_ + z_};

  EXPECT_FALSE(add1.Less(add1));
  EXPECT_TRUE(add1.Less(add2));
  EXPECT_TRUE(add1.Less(add3));
  EXPECT_TRUE(add1.Less(add4));
  EXPECT_FALSE(add2.Less(add1));
  EXPECT_FALSE(add2.Less(add2));
  EXPECT_TRUE(add2.Less(add3));
  EXPECT_TRUE(add2.Less(add4));
  EXPECT_FALSE(add3.Less(add1));
  EXPECT_FALSE(add3.Less(add2));
  EXPECT_FALSE(add3.Less(add3));
  EXPECT_TRUE(add3.Less(add4));
  EXPECT_FALSE(add4.Less(add1));
  EXPECT_FALSE(add4.Less(add2));
  EXPECT_FALSE(add4.Less(add3));
  EXPECT_FALSE(add4.Less(add4));
}

TEST_F(SymbolicExpressionTest, LessSub) {
  const Expression sub1{c1_ - x_ - y_};
  const Expression sub2{c1_ - y_ - z_};
  const Expression sub3{c3_ - x_ - y_};
  const Expression sub4{c3_ - y_ - z_};

  EXPECT_FALSE(sub1.Less(sub1));
  EXPECT_TRUE(sub1.Less(sub2));
  EXPECT_TRUE(sub1.Less(sub3));
  EXPECT_TRUE(sub1.Less(sub4));
  EXPECT_FALSE(sub2.Less(sub1));
  EXPECT_FALSE(sub2.Less(sub2));
  EXPECT_TRUE(sub2.Less(sub3));
  EXPECT_TRUE(sub2.Less(sub4));
  EXPECT_FALSE(sub3.Less(sub1));
  EXPECT_FALSE(sub3.Less(sub2));
  EXPECT_FALSE(sub3.Less(sub3));
  EXPECT_TRUE(sub3.Less(sub4));
  EXPECT_FALSE(sub4.Less(sub1));
  EXPECT_FALSE(sub4.Less(sub2));
  EXPECT_FALSE(sub4.Less(sub3));
  EXPECT_FALSE(sub4.Less(sub4));
}

TEST_F(SymbolicExpressionTest, LessMul) {
  const Expression mul1{c1_ * x_ * y_};
  const Expression mul2{c1_ * y_ * z_};
  const Expression mul3{c3_ * x_ * y_};
  const Expression mul4{c3_ * y_ * z_};

  EXPECT_FALSE(mul1.Less(mul1));
  EXPECT_TRUE(mul1.Less(mul2));
  EXPECT_TRUE(mul1.Less(mul3));
  EXPECT_TRUE(mul1.Less(mul4));
  EXPECT_FALSE(mul2.Less(mul1));
  EXPECT_FALSE(mul2.Less(mul2));
  EXPECT_TRUE(mul2.Less(mul3));
  EXPECT_TRUE(mul2.Less(mul4));
  EXPECT_FALSE(mul3.Less(mul1));
  EXPECT_FALSE(mul3.Less(mul2));
  EXPECT_FALSE(mul3.Less(mul3));
  EXPECT_TRUE(mul3.Less(mul4));
  EXPECT_FALSE(mul4.Less(mul1));
  EXPECT_FALSE(mul4.Less(mul2));
  EXPECT_FALSE(mul4.Less(mul3));
  EXPECT_FALSE(mul4.Less(mul4));
}

TEST_F(SymbolicExpressionTest, LessDiv) {
  const Expression div1{x_ / y_};
  const Expression div2{x_ / z_};
  const Expression div3{y_ / z_};

  EXPECT_FALSE(div1.Less(div1));
  EXPECT_TRUE(div1.Less(div2));
  EXPECT_TRUE(div1.Less(div3));
  EXPECT_FALSE(div2.Less(div1));
  EXPECT_FALSE(div2.Less(div2));
  EXPECT_TRUE(div2.Less(div3));
  EXPECT_FALSE(div3.Less(div1));
  EXPECT_FALSE(div3.Less(div2));
  EXPECT_FALSE(div3.Less(div3));
}

TEST_F(SymbolicExpressionTest, LessLog) {
  const Expression log1{log(x_)};
  const Expression log2{log(y_)};
  const Expression log3{log(x_plus_y_)};
  const Expression log4{log(x_plus_z_)};

  EXPECT_FALSE(log1.Less(log1));
  EXPECT_TRUE(log1.Less(log2));
  EXPECT_TRUE(log1.Less(log3));
  EXPECT_TRUE(log1.Less(log4));
  EXPECT_FALSE(log2.Less(log1));
  EXPECT_FALSE(log2.Less(log2));
  EXPECT_TRUE(log2.Less(log3));
  EXPECT_TRUE(log2.Less(log4));
  EXPECT_FALSE(log3.Less(log1));
  EXPECT_FALSE(log3.Less(log2));
  EXPECT_FALSE(log3.Less(log3));
  EXPECT_TRUE(log3.Less(log4));
  EXPECT_FALSE(log4.Less(log1));
  EXPECT_FALSE(log4.Less(log2));
  EXPECT_FALSE(log4.Less(log3));
  EXPECT_FALSE(log4.Less(log4));
}

TEST_F(SymbolicExpressionTest, LessAbs) {
  const Expression abs1{abs(x_)};
  const Expression abs2{abs(y_)};
  const Expression abs3{abs(x_plus_y_)};
  const Expression abs4{abs(x_plus_z_)};

  EXPECT_FALSE(abs1.Less(abs1));
  EXPECT_TRUE(abs1.Less(abs2));
  EXPECT_TRUE(abs1.Less(abs3));
  EXPECT_TRUE(abs1.Less(abs4));
  EXPECT_FALSE(abs2.Less(abs1));
  EXPECT_FALSE(abs2.Less(abs2));
  EXPECT_TRUE(abs2.Less(abs3));
  EXPECT_TRUE(abs2.Less(abs4));
  EXPECT_FALSE(abs3.Less(abs1));
  EXPECT_FALSE(abs3.Less(abs2));
  EXPECT_FALSE(abs3.Less(abs3));
  EXPECT_TRUE(abs3.Less(abs4));
  EXPECT_FALSE(abs4.Less(abs1));
  EXPECT_FALSE(abs4.Less(abs2));
  EXPECT_FALSE(abs4.Less(abs3));
  EXPECT_FALSE(abs4.Less(abs4));
}

TEST_F(SymbolicExpressionTest, LessExp) {
  const Expression exp1{exp(x_)};
  const Expression exp2{exp(y_)};
  const Expression exp3{exp(x_plus_y_)};
  const Expression exp4{exp(x_plus_z_)};

  EXPECT_FALSE(exp1.Less(exp1));
  EXPECT_TRUE(exp1.Less(exp2));
  EXPECT_TRUE(exp1.Less(exp3));
  EXPECT_TRUE(exp1.Less(exp4));
  EXPECT_FALSE(exp2.Less(exp1));
  EXPECT_FALSE(exp2.Less(exp2));
  EXPECT_TRUE(exp2.Less(exp3));
  EXPECT_TRUE(exp2.Less(exp4));
  EXPECT_FALSE(exp3.Less(exp1));
  EXPECT_FALSE(exp3.Less(exp2));
  EXPECT_FALSE(exp3.Less(exp3));
  EXPECT_TRUE(exp3.Less(exp4));
  EXPECT_FALSE(exp4.Less(exp1));
  EXPECT_FALSE(exp4.Less(exp2));
  EXPECT_FALSE(exp4.Less(exp3));
  EXPECT_FALSE(exp4.Less(exp4));
}

TEST_F(SymbolicExpressionTest, LessSqrt) {
  const Expression sqrt1{sqrt(x_)};
  const Expression sqrt2{sqrt(y_)};
  const Expression sqrt3{sqrt(x_plus_y_)};
  const Expression sqrt4{sqrt(x_plus_z_)};

  EXPECT_FALSE(sqrt1.Less(sqrt1));
  EXPECT_TRUE(sqrt1.Less(sqrt2));
  EXPECT_TRUE(sqrt1.Less(sqrt3));
  EXPECT_TRUE(sqrt1.Less(sqrt4));
  EXPECT_FALSE(sqrt2.Less(sqrt1));
  EXPECT_FALSE(sqrt2.Less(sqrt2));
  EXPECT_TRUE(sqrt2.Less(sqrt3));
  EXPECT_TRUE(sqrt2.Less(sqrt4));
  EXPECT_FALSE(sqrt3.Less(sqrt1));
  EXPECT_FALSE(sqrt3.Less(sqrt2));
  EXPECT_FALSE(sqrt3.Less(sqrt3));
  EXPECT_TRUE(sqrt3.Less(sqrt4));
  EXPECT_FALSE(sqrt4.Less(sqrt1));
  EXPECT_FALSE(sqrt4.Less(sqrt2));
  EXPECT_FALSE(sqrt4.Less(sqrt3));
  EXPECT_FALSE(sqrt4.Less(sqrt4));
}

TEST_F(SymbolicExpressionTest, LessSin) {
  const Expression sin1{sin(x_)};
  const Expression sin2{sin(y_)};
  const Expression sin3{sin(x_plus_y_)};
  const Expression sin4{sin(x_plus_z_)};

  EXPECT_FALSE(sin1.Less(sin1));
  EXPECT_TRUE(sin1.Less(sin2));
  EXPECT_TRUE(sin1.Less(sin3));
  EXPECT_TRUE(sin1.Less(sin4));
  EXPECT_FALSE(sin2.Less(sin1));
  EXPECT_FALSE(sin2.Less(sin2));
  EXPECT_TRUE(sin2.Less(sin3));
  EXPECT_TRUE(sin2.Less(sin4));
  EXPECT_FALSE(sin3.Less(sin1));
  EXPECT_FALSE(sin3.Less(sin2));
  EXPECT_FALSE(sin3.Less(sin3));
  EXPECT_TRUE(sin3.Less(sin4));
  EXPECT_FALSE(sin4.Less(sin1));
  EXPECT_FALSE(sin4.Less(sin2));
  EXPECT_FALSE(sin4.Less(sin3));
  EXPECT_FALSE(sin4.Less(sin4));
}

TEST_F(SymbolicExpressionTest, LessCos) {
  const Expression cos1{cos(x_)};
  const Expression cos2{cos(y_)};
  const Expression cos3{cos(x_plus_y_)};
  const Expression cos4{cos(x_plus_z_)};

  EXPECT_FALSE(cos1.Less(cos1));
  EXPECT_TRUE(cos1.Less(cos2));
  EXPECT_TRUE(cos1.Less(cos3));
  EXPECT_TRUE(cos1.Less(cos4));
  EXPECT_FALSE(cos2.Less(cos1));
  EXPECT_FALSE(cos2.Less(cos2));
  EXPECT_TRUE(cos2.Less(cos3));
  EXPECT_TRUE(cos2.Less(cos4));
  EXPECT_FALSE(cos3.Less(cos1));
  EXPECT_FALSE(cos3.Less(cos2));
  EXPECT_FALSE(cos3.Less(cos3));
  EXPECT_TRUE(cos3.Less(cos4));
  EXPECT_FALSE(cos4.Less(cos1));
  EXPECT_FALSE(cos4.Less(cos2));
  EXPECT_FALSE(cos4.Less(cos3));
  EXPECT_FALSE(cos4.Less(cos4));
}

TEST_F(SymbolicExpressionTest, LessTan) {
  const Expression tan1{tan(x_)};
  const Expression tan2{tan(y_)};
  const Expression tan3{tan(x_plus_y_)};
  const Expression tan4{tan(x_plus_z_)};

  EXPECT_FALSE(tan1.Less(tan1));
  EXPECT_TRUE(tan1.Less(tan2));
  EXPECT_TRUE(tan1.Less(tan3));
  EXPECT_TRUE(tan1.Less(tan4));
  EXPECT_FALSE(tan2.Less(tan1));
  EXPECT_FALSE(tan2.Less(tan2));
  EXPECT_TRUE(tan2.Less(tan3));
  EXPECT_TRUE(tan2.Less(tan4));
  EXPECT_FALSE(tan3.Less(tan1));
  EXPECT_FALSE(tan3.Less(tan2));
  EXPECT_FALSE(tan3.Less(tan3));
  EXPECT_TRUE(tan3.Less(tan4));
  EXPECT_FALSE(tan4.Less(tan1));
  EXPECT_FALSE(tan4.Less(tan2));
  EXPECT_FALSE(tan4.Less(tan3));
  EXPECT_FALSE(tan4.Less(tan4));
}

TEST_F(SymbolicExpressionTest, LessAsin) {
  const Expression asin1{asin(x_)};
  const Expression asin2{asin(y_)};
  const Expression asin3{asin(x_plus_y_)};
  const Expression asin4{asin(x_plus_z_)};

  EXPECT_FALSE(asin1.Less(asin1));
  EXPECT_TRUE(asin1.Less(asin2));
  EXPECT_TRUE(asin1.Less(asin3));
  EXPECT_TRUE(asin1.Less(asin4));
  EXPECT_FALSE(asin2.Less(asin1));
  EXPECT_FALSE(asin2.Less(asin2));
  EXPECT_TRUE(asin2.Less(asin3));
  EXPECT_TRUE(asin2.Less(asin4));
  EXPECT_FALSE(asin3.Less(asin1));
  EXPECT_FALSE(asin3.Less(asin2));
  EXPECT_FALSE(asin3.Less(asin3));
  EXPECT_TRUE(asin3.Less(asin4));
  EXPECT_FALSE(asin4.Less(asin1));
  EXPECT_FALSE(asin4.Less(asin2));
  EXPECT_FALSE(asin4.Less(asin3));
  EXPECT_FALSE(asin4.Less(asin4));
}

TEST_F(SymbolicExpressionTest, LessAcos) {
  const Expression acos1{acos(x_)};
  const Expression acos2{acos(y_)};
  const Expression acos3{acos(x_plus_y_)};
  const Expression acos4{acos(x_plus_z_)};

  EXPECT_FALSE(acos1.Less(acos1));
  EXPECT_TRUE(acos1.Less(acos2));
  EXPECT_TRUE(acos1.Less(acos3));
  EXPECT_TRUE(acos1.Less(acos4));
  EXPECT_FALSE(acos2.Less(acos1));
  EXPECT_FALSE(acos2.Less(acos2));
  EXPECT_TRUE(acos2.Less(acos3));
  EXPECT_TRUE(acos2.Less(acos4));
  EXPECT_FALSE(acos3.Less(acos1));
  EXPECT_FALSE(acos3.Less(acos2));
  EXPECT_FALSE(acos3.Less(acos3));
  EXPECT_TRUE(acos3.Less(acos4));
  EXPECT_FALSE(acos4.Less(acos1));
  EXPECT_FALSE(acos4.Less(acos2));
  EXPECT_FALSE(acos4.Less(acos3));
  EXPECT_FALSE(acos4.Less(acos4));
}

TEST_F(SymbolicExpressionTest, LessAtan) {
  const Expression atan1{atan(x_)};
  const Expression atan2{atan(y_)};
  const Expression atan3{atan(x_plus_y_)};
  const Expression atan4{atan(x_plus_z_)};

  EXPECT_FALSE(atan1.Less(atan1));
  EXPECT_TRUE(atan1.Less(atan2));
  EXPECT_TRUE(atan1.Less(atan3));
  EXPECT_TRUE(atan1.Less(atan4));
  EXPECT_FALSE(atan2.Less(atan1));
  EXPECT_FALSE(atan2.Less(atan2));
  EXPECT_TRUE(atan2.Less(atan3));
  EXPECT_TRUE(atan2.Less(atan4));
  EXPECT_FALSE(atan3.Less(atan1));
  EXPECT_FALSE(atan3.Less(atan2));
  EXPECT_FALSE(atan3.Less(atan3));
  EXPECT_TRUE(atan3.Less(atan4));
  EXPECT_FALSE(atan4.Less(atan1));
  EXPECT_FALSE(atan4.Less(atan2));
  EXPECT_FALSE(atan4.Less(atan3));
  EXPECT_FALSE(atan4.Less(atan4));
}

TEST_F(SymbolicExpressionTest, LessSinh) {
  const Expression sinh1{sinh(x_)};
  const Expression sinh2{sinh(y_)};
  const Expression sinh3{sinh(x_plus_y_)};
  const Expression sinh4{sinh(x_plus_z_)};

  EXPECT_FALSE(sinh1.Less(sinh1));
  EXPECT_TRUE(sinh1.Less(sinh2));
  EXPECT_TRUE(sinh1.Less(sinh3));
  EXPECT_TRUE(sinh1.Less(sinh4));
  EXPECT_FALSE(sinh2.Less(sinh1));
  EXPECT_FALSE(sinh2.Less(sinh2));
  EXPECT_TRUE(sinh2.Less(sinh3));
  EXPECT_TRUE(sinh2.Less(sinh4));
  EXPECT_FALSE(sinh3.Less(sinh1));
  EXPECT_FALSE(sinh3.Less(sinh2));
  EXPECT_FALSE(sinh3.Less(sinh3));
  EXPECT_TRUE(sinh3.Less(sinh4));
  EXPECT_FALSE(sinh4.Less(sinh1));
  EXPECT_FALSE(sinh4.Less(sinh2));
  EXPECT_FALSE(sinh4.Less(sinh3));
  EXPECT_FALSE(sinh4.Less(sinh4));
}

TEST_F(SymbolicExpressionTest, LessCosh) {
  const Expression cosh1{cosh(x_)};
  const Expression cosh2{cosh(y_)};
  const Expression cosh3{cosh(x_plus_y_)};
  const Expression cosh4{cosh(x_plus_z_)};

  EXPECT_FALSE(cosh1.Less(cosh1));
  EXPECT_TRUE(cosh1.Less(cosh2));
  EXPECT_TRUE(cosh1.Less(cosh3));
  EXPECT_TRUE(cosh1.Less(cosh4));
  EXPECT_FALSE(cosh2.Less(cosh1));
  EXPECT_FALSE(cosh2.Less(cosh2));
  EXPECT_TRUE(cosh2.Less(cosh3));
  EXPECT_TRUE(cosh2.Less(cosh4));
  EXPECT_FALSE(cosh3.Less(cosh1));
  EXPECT_FALSE(cosh3.Less(cosh2));
  EXPECT_FALSE(cosh3.Less(cosh3));
  EXPECT_TRUE(cosh3.Less(cosh4));
  EXPECT_FALSE(cosh4.Less(cosh1));
  EXPECT_FALSE(cosh4.Less(cosh2));
  EXPECT_FALSE(cosh4.Less(cosh3));
  EXPECT_FALSE(cosh4.Less(cosh4));
}

TEST_F(SymbolicExpressionTest, LessTanh) {
  const Expression tanh1{tanh(x_)};
  const Expression tanh2{tanh(y_)};
  const Expression tanh3{tanh(x_plus_y_)};
  const Expression tanh4{tanh(x_plus_z_)};

  EXPECT_FALSE(tanh1.Less(tanh1));
  EXPECT_TRUE(tanh1.Less(tanh2));
  EXPECT_TRUE(tanh1.Less(tanh3));
  EXPECT_TRUE(tanh1.Less(tanh4));
  EXPECT_FALSE(tanh2.Less(tanh1));
  EXPECT_FALSE(tanh2.Less(tanh2));
  EXPECT_TRUE(tanh2.Less(tanh3));
  EXPECT_TRUE(tanh2.Less(tanh4));
  EXPECT_FALSE(tanh3.Less(tanh1));
  EXPECT_FALSE(tanh3.Less(tanh2));
  EXPECT_FALSE(tanh3.Less(tanh3));
  EXPECT_TRUE(tanh3.Less(tanh4));
  EXPECT_FALSE(tanh4.Less(tanh1));
  EXPECT_FALSE(tanh4.Less(tanh2));
  EXPECT_FALSE(tanh4.Less(tanh3));
  EXPECT_FALSE(tanh4.Less(tanh4));
}

TEST_F(SymbolicExpressionTest, LessPow) {
  const Expression pow1{pow(x_, y_)};
  const Expression pow2{pow(x_, z_)};
  const Expression pow3{pow(y_, z_)};

  EXPECT_FALSE(pow1.Less(pow1));
  EXPECT_TRUE(pow1.Less(pow2));
  EXPECT_TRUE(pow1.Less(pow3));
  EXPECT_FALSE(pow2.Less(pow1));
  EXPECT_FALSE(pow2.Less(pow2));
  EXPECT_TRUE(pow2.Less(pow3));
  EXPECT_FALSE(pow3.Less(pow1));
  EXPECT_FALSE(pow3.Less(pow2));
  EXPECT_FALSE(pow3.Less(pow3));
}

TEST_F(SymbolicExpressionTest, LessAtan2) {
  const Expression atan2_1{atan2(x_, y_)};
  const Expression atan2_2{atan2(x_, z_)};
  const Expression atan2_3{atan2(y_, z_)};

  EXPECT_FALSE(atan2_1.Less(atan2_1));
  EXPECT_TRUE(atan2_1.Less(atan2_2));
  EXPECT_TRUE(atan2_1.Less(atan2_3));
  EXPECT_FALSE(atan2_2.Less(atan2_1));
  EXPECT_FALSE(atan2_2.Less(atan2_2));
  EXPECT_TRUE(atan2_2.Less(atan2_3));
  EXPECT_FALSE(atan2_3.Less(atan2_1));
  EXPECT_FALSE(atan2_3.Less(atan2_2));
  EXPECT_FALSE(atan2_3.Less(atan2_3));
}

TEST_F(SymbolicExpressionTest, LessMin) {
  const Expression min1{min(x_, y_)};
  const Expression min2{min(x_, z_)};
  const Expression min3{min(y_, z_)};

  EXPECT_FALSE(min1.Less(min1));
  EXPECT_TRUE(min1.Less(min2));
  EXPECT_TRUE(min1.Less(min3));
  EXPECT_FALSE(min2.Less(min1));
  EXPECT_FALSE(min2.Less(min2));
  EXPECT_TRUE(min2.Less(min3));
  EXPECT_FALSE(min3.Less(min1));
  EXPECT_FALSE(min3.Less(min2));
  EXPECT_FALSE(min3.Less(min3));
}

TEST_F(SymbolicExpressionTest, LessMax) {
  const Expression max1{max(x_, y_)};
  const Expression max2{max(x_, z_)};
  const Expression max3{max(y_, z_)};

  EXPECT_FALSE(max1.Less(max1));
  EXPECT_TRUE(max1.Less(max2));
  EXPECT_TRUE(max1.Less(max3));
  EXPECT_FALSE(max2.Less(max1));
  EXPECT_FALSE(max2.Less(max2));
  EXPECT_TRUE(max2.Less(max3));
  EXPECT_FALSE(max3.Less(max1));
  EXPECT_FALSE(max3.Less(max2));
  EXPECT_FALSE(max3.Less(max3));
}

TEST_F(SymbolicExpressionTest, Variable) {
  EXPECT_EQ(x_.to_string(), var_x_.get_name());
  EXPECT_EQ(y_.to_string(), var_y_.get_name());
  EXPECT_EQ(z_.to_string(), var_z_.get_name());
  EXPECT_PRED2(ExpEqual, x_, x_);
  EXPECT_PRED2(ExpNotEqual, x_, y_);
  EXPECT_PRED2(ExpNotEqual, x_, z_);
  EXPECT_PRED2(ExpNotEqual, y_, x_);
  EXPECT_PRED2(ExpEqual, y_, y_);
  EXPECT_PRED2(ExpNotEqual, y_, z_);
  EXPECT_PRED2(ExpNotEqual, z_, x_);
  EXPECT_PRED2(ExpNotEqual, z_, y_);
  EXPECT_PRED2(ExpEqual, z_, z_);
}

TEST_F(SymbolicExpressionTest, Constant) {
  EXPECT_EQ(c1_.Evaluate(), -10);
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
  EXPECT_PRED2(ExpNotEqual, c3_, -c3_);
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), -(-c3_).Evaluate());
  EXPECT_PRED2(ExpEqual, c3_, -(-c3_));
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), (-(-c3_)).Evaluate());
  const Expression e{x_ + y_};
  const Environment env{{var_x_, 1.0}, {var_y_, 2.0}};
  // (x + y) and -(-(x + y)) are structurally equal (after simplification)
  EXPECT_PRED2(ExpEqual, e, -(-e));
  // and their evaluations should be the same.
  EXPECT_DOUBLE_EQ(e.Evaluate(env), (-(-e)).Evaluate(env));

  EXPECT_PRED2(ExpEqual, -(x_plus_y_ + x_plus_z_ + pi_),
               -x_plus_y_ + (-x_plus_z_) + (-pi_));
}

TEST_F(SymbolicExpressionTest, Add1) {
  EXPECT_PRED2(ExpEqual, c3_ + zero_, c3_);
  EXPECT_EQ((c3_ + zero_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, zero_ + c3_, c3_);
  EXPECT_EQ((zero_ + c3_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, 0.0 + c3_, c3_);
  EXPECT_EQ((0.0 + c3_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, c3_ + 0.0, c3_);
  EXPECT_EQ((c3_ + 0.0).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, c3_ + c4_, Expression{3.14159 + -2.718});
  EXPECT_EQ((c3_ + c4_).to_string(), Expression{3.14159 + -2.718}.to_string());
  EXPECT_PRED2(ExpEqual, c3_ + x_, 3.14159 + x_);
  EXPECT_EQ((c3_ + x_).to_string(), (3.14159 + x_).to_string());
  EXPECT_PRED2(ExpEqual, x_ + c3_, x_ + 3.14159);
  EXPECT_EQ((x_ + c3_).to_string(), (x_ + 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Add2) {
  Expression e1{x_ + y_};
  Expression e2{e1 + e1};
  const auto str_rep_e2(e2.to_string());
  EXPECT_PRED2(ExpEqual, e2, 2 * x_ + 2 * y_);
  e1 += z_;
  EXPECT_PRED2(ExpEqual, e1, x_ + y_ + z_);
  EXPECT_EQ(e2.to_string(), str_rep_e2);  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Inc1) {
  // Prefix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};
  EXPECT_PRED2(ExpEqual, x, x_prime);
  EXPECT_PRED2(ExpEqual, x++, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime++);
  EXPECT_PRED2(ExpEqual, x, x_prime);
}

TEST_F(SymbolicExpressionTest, Inc2) {
  // Postfix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExpEqual, x, x_prime);
  EXPECT_PRED2(ExpNotEqual, ++x, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime);
  EXPECT_PRED2(ExpEqual, x, ++x_prime);
  EXPECT_PRED2(ExpEqual, x, x_prime);
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
  EXPECT_PRED2(ExpEqual, c3_ - zero_, c3_);
  EXPECT_EQ((c3_ - zero_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, zero_ - c3_, -c3_);
  EXPECT_EQ((zero_ - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_PRED2(ExpEqual, 0.0 - c3_, -c3_);
  EXPECT_EQ((0.0 - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_PRED2(ExpEqual, 0.0 - c3_, (-1 * c3_));
  EXPECT_EQ((0.0 - c3_).to_string(), (-1 * c3_).to_string());
  EXPECT_PRED2(ExpEqual, c3_ - 0.0, c3_);
  EXPECT_EQ((c3_ - 0.0).to_string(), c3_.to_string());
  EXPECT_PRED2(ExpEqual, c3_ - c4_, Expression{3.14159 - -2.718});
  EXPECT_EQ((c3_ - c4_).to_string(), Expression{3.14159 - -2.718}.to_string());
  EXPECT_PRED2(ExpEqual, c3_ - x_, 3.14159 - x_);
  EXPECT_EQ((c3_ - x_).to_string(), (3.14159 - x_).to_string());
  EXPECT_PRED2(ExpEqual, x_ - c3_, x_ - 3.14159);
  EXPECT_EQ((x_ - c3_).to_string(), (x_ - 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Sub2) {
  Expression e1{x_ - y_};
  const Expression e2{x_ - z_};
  const Expression e3{e1 - e2};
  const auto str_rep_e3(e3.to_string());
  e1 -= z_;
  EXPECT_PRED2(ExpEqual, e1, x_ - y_ - z_);
  EXPECT_EQ(e3.to_string(), str_rep_e3);  // e3 doesn't change.
}

TEST_F(SymbolicExpressionTest, Sub3) {
  const Expression e1{x_ - y_};
  const Expression e2{x_ - y_};
  const Expression e3{e1 - e2};
  EXPECT_PRED2(ExpEqual, e1, e2);
  EXPECT_EQ(e3.to_string(), "0");  // simplified
}

TEST_F(SymbolicExpressionTest, Dec1) {
  // Postfix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExpEqual, x, x_prime);
  EXPECT_PRED2(ExpEqual, x--, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime--);
  EXPECT_PRED2(ExpEqual, x, x_prime);
}

TEST_F(SymbolicExpressionTest, Dec2) {
  // Prefix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExpEqual, x, x_prime);
  EXPECT_PRED2(ExpNotEqual, --x, x_prime);
  EXPECT_PRED2(ExpNotEqual, x, x_prime);
  EXPECT_PRED2(ExpEqual, x, --x_prime);
  EXPECT_PRED2(ExpEqual, x, x_prime);
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

TEST_F(SymbolicExpressionTest, Mul1) {
  EXPECT_PRED2(ExpEqual, c3_ * zero_, zero_);
  EXPECT_PRED2(ExpEqual, zero_ * c3_, zero_);
  EXPECT_PRED2(ExpEqual, c3_ * 0.0, zero_);
  EXPECT_PRED2(ExpEqual, 0.0 * c3_, zero_);

  EXPECT_PRED2(ExpEqual, c3_ * one_, c3_);
  EXPECT_PRED2(ExpEqual, one_ * c3_, c3_);
  EXPECT_PRED2(ExpEqual, 1.0 * c3_, c3_);
  EXPECT_PRED2(ExpEqual, c3_ * 1.0, c3_);

  EXPECT_PRED2(ExpEqual, c3_ * c4_, Expression{3.14159 * -2.718});
  EXPECT_PRED2(ExpEqual, c3_ * x_, (3.14159 * x_));
  EXPECT_PRED2(ExpEqual, x_ * c3_, (x_ * 3.14159));
}

TEST_F(SymbolicExpressionTest, Mul2) {
  Expression e1{x_ * y_};
  Expression e2{e1 * e1};
  EXPECT_EQ(e1.to_string(), "(x * y)");
  EXPECT_PRED2(ExpEqual, e2, pow(x_, 2) * pow(y_, 2));
  e1 *= z_;
  EXPECT_PRED2(ExpEqual, e1, x_ * y_ * z_);
  EXPECT_PRED2(ExpEqual, e2, pow(x_, 2) * pow(y_, 2));  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Mul3) {
  const Expression e1{x_ * x_ * x_ * x_};
  const Expression e2{(x_ * x_) * (x_ * x_)};
  const Expression e3{x_ * (x_ * x_ * x_)};
  const Expression e4{pow(x_, 4)};
  EXPECT_PRED2(ExpEqual, e1, e4);
  EXPECT_PRED2(ExpEqual, e2, e4);
  EXPECT_PRED2(ExpEqual, e3, e4);
}

TEST_F(SymbolicExpressionTest, Mul4) {
  // x * y * y * x = x^2 * y^2
  EXPECT_PRED2(ExpEqual, x_ * y_ * y_ * x_, pow(x_, 2) * pow(y_, 2));
  // (x * y * y * x) * (x * y * x * y) = x^4 * y^4
  EXPECT_PRED2(ExpEqual, (x_ * y_ * y_ * x_) * (x_ * y_ * x_ * y_),
               pow(x_, 4) * pow(y_, 4));
  EXPECT_PRED2(ExpEqual, 3 * (4 * x_), (3 * 4) * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * 4, (3 * 4) * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * (4 * x_), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExpEqual, (x_ * 3) * (x_ * 4), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * (4 * y_), (3 * 4) * x_ * y_);
}

TEST_F(SymbolicExpressionTest, Mul5) {
  // x^y * x^z = x^(y + z)
  EXPECT_PRED2(ExpEqual, pow(x_, y_) * pow(x_, z_), pow(x_, y_ + z_));

  // x^y * x = x^(y + 1)
  EXPECT_PRED2(ExpEqual, pow(x_, y_) * x_, pow(x_, y_ + 1));

  // x * pow(x, y) = x^(1 + y)
  EXPECT_PRED2(ExpEqual, x_ * pow(x_, y_), pow(x_, 1 + y_));

  // x * y * y * x = x^2 * y^2
  EXPECT_PRED2(ExpEqual, x_ * y_ * y_ * x_, pow(x_, 2) * pow(y_, 2));
  // (x * y * y * x) * (x * y * x * y) = x^4 * y^4
  EXPECT_PRED2(ExpEqual, (x_ * y_ * y_ * x_) * (x_ * y_ * x_ * y_),
               pow(x_, 4) * pow(y_, 4));
  EXPECT_PRED2(ExpEqual, 3 * (4 * x_), (3 * 4) * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * 4, (3 * 4) * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * (4 * x_), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExpEqual, (x_ * 3) * (x_ * 4), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExpEqual, (3 * x_) * (4 * y_), (3 * 4) * x_ * y_);
}

TEST_F(SymbolicExpressionTest, AddMul1) {
  const Expression e1{(x_ * y_ * y_ * x_) + (x_ * y_ * x_ * y_)};
  EXPECT_PRED2(ExpEqual, e1, 2 * pow(x_, 2) * pow(y_, 2));

  const Expression e2{x_ + y_ + (-x_)};
  EXPECT_PRED2(ExpEqual, e2, y_);

  const Expression e3{(2 * x_) + (3 * x_)};
  EXPECT_PRED2(ExpEqual, e3, 5 * x_);

  const Expression e4{(x_ * 2 * x_) + (x_ * x_ * 3)};
  EXPECT_PRED2(ExpEqual, e4, 5 * x_ * x_);
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
  unordered_set<Expression, hash_value<Expression>> uset;
  uset.emplace(Expression{Variable{"a"}});
  uset.emplace(Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::unordered_map.
GTEST_TEST(ExpressionTest, CompatibleWithUnorderedMap) {
  unordered_map<Expression, Expression, hash_value<Expression>> umap;
  umap.emplace(Expression{Variable{"a"}}, Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::set.
GTEST_TEST(ExpressionTest, CompatibleWithSet) {
  set<Expression> set;
  set.emplace(Expression{Variable{"a"}});
  set.emplace(Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::map.
GTEST_TEST(ExpressionTest, CompatibleWithMap) {
  map<Expression, Expression> map;
  map.emplace(Expression{Variable{"a"}}, Expression{Variable{"b"}});
}

// This test checks whether symbolic::Expression is compatible with
// std::vector.
GTEST_TEST(ExpressionTest, CompatibleWithVector) {
  vector<Expression> vec;
  vec.push_back(Expression{123.0});
}

GTEST_TEST(ExpressionTest, NoThrowMoveConstructible) {
  // make sure that symbolic::Expression is nothrow move-constructible so that
  // it can be moved (not copied) when a STL container (i.e. vector<Expression>)
  // is resized.
  EXPECT_TRUE(std::is_nothrow_move_constructible<Expression>::value);
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

TEST_F(SymbolicExpressionTest, Sqrt1) {
  // sqrt(x * x) => |x|
  EXPECT_PRED2(ExpEqual, sqrt(x_plus_y_ * x_plus_y_), abs(x_plus_y_));
}

TEST_F(SymbolicExpressionTest, Sqrt2) {
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

TEST_F(SymbolicExpressionTest, Pow1) {
  // pow(x, 0.0) => 1.0
  EXPECT_PRED2(ExpEqual, pow(x_plus_y_, Expression::Zero()), Expression::One());
  // pow(x, 1.0) => x
  EXPECT_PRED2(ExpEqual, pow(x_plus_y_, Expression::One()), x_plus_y_);
  // sqrt(x) * sqrt(x) => pow(sqrt(x), 2) => x
  EXPECT_PRED2(ExpEqual, (sqrt(x_plus_y_) * sqrt(x_plus_y_)), x_plus_y_);
  // (x^2)^3 => x^(2*3)
  EXPECT_PRED2(ExpEqual, pow(pow(x_, 2), 3), pow(x_, 2 * 3));
  // (x^y)^z => x^(y*z)
  EXPECT_PRED2(ExpEqual, pow(pow(x_, y_), z_), pow(x_, y_ * z_));
}

TEST_F(SymbolicExpressionTest, Pow2) {
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
  EXPECT_PRED2(ExpEqual, min(x_plus_y_, x_plus_y_), x_plus_y_);
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
  EXPECT_PRED2(ExpEqual, max(x_plus_y_, x_plus_y_), x_plus_y_);
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
  EXPECT_PRED2(ExpEqual, e1, e1_copy);
  EXPECT_PRED2(ExpEqual, e2, e2_copy);

  swap(e1, e2);

  // After Swap.
  EXPECT_PRED2(ExpEqual, e1, e2_copy);
  EXPECT_PRED2(ExpEqual, e2, e1_copy);
}

TEST_F(SymbolicExpressionTest, ToString) {
  const Expression e1{sin(x_ + y_ * z_)};
  const Expression e2{cos(x_ * x_ + pow(y_, 2) * z_)};

  EXPECT_EQ(e1.to_string(), "sin((x + (y * z)))");
  EXPECT_EQ(e2.to_string(), "cos(((pow(y, 2) * z) + pow(x, 2)))");
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
  auto const M(A_ + A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ + x_), (one_ + one_),
                (y_ + y_), (neg_one_ + neg_one_),
                (z_ + z_), (pi_ + pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub1) {
  auto const M(A_ - A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenSub2) {
  auto const M(A_ - C_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ - z_), (one_ - two_),
                (y_ - x_), (neg_one_ - e_),
                (z_ - y_), (pi_ - pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);  // should be all zero.
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul1) {
  auto const M(A_ * B_);
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
  auto const M(B_ * A_);
  Eigen::Matrix<Expression, 2, 2> M_expected;
  // clang-format off
  M_expected <<
    (x_ * x_ + (y_ * y_ + z_ * z_)),    (x_ + (-y_ + z_ * pi_)),
    (e_ * x_ + (pi_ * y_ + two_ * z_)), (e_ * one_ + pi_ * - one_ + two_ * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul3) {
  auto const M(2.0 * A_);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (2 * x_), (2 * one_),
                (2 * y_), (2 * neg_one_),
                (2 * z_), (2 * pi_);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenMul4) {
  auto const M(A_ * 2.0);
  Eigen::Matrix<Expression, 3, 2> M_expected;
  // clang-format off
  M_expected << (x_ * 2), (one_ * 2),
                (y_ * 2), (neg_one_ * 2),
                (z_ * 2), (pi_ * 2);
  // clang-format on
  EXPECT_EQ(M, M_expected);
}

TEST_F(SymbolicExpressionMatrixTest, EigenDiv) {
  auto const M(A_ / 2.0);
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
