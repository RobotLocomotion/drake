#include "drake/common/symbolic_expression.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/hash.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/common/test/symbolic_test_util.h"

using std::count_if;
using std::domain_error;
using std::equal_to;
using std::map;
using std::ostringstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::string;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;
using test::ExprLess;
using test::ExprNotEqual;
using test::ExprNotLess;

// Checks if a given 'expressions' is ordered by Expression::Less.
static void CheckOrdering(const vector<Expression>& expressions) {
  for (size_t i{0}; i < expressions.size(); ++i) {
    for (size_t j{0}; j < expressions.size(); ++j) {
      if (i < j) {
        EXPECT_PRED2(ExprLess, expressions[i], expressions[j])
            << "(Expressions[" << i << "] = " << expressions[i] << ")"
            << " is not less than "
            << "(Expressions[" << j << "] = " << expressions[j] << ")";
        EXPECT_PRED2(ExprNotLess, expressions[j], expressions[i])
            << "(Expressions[" << j << "] = " << expressions[j] << ")"
            << " is less than "
            << "(Expressions[" << i << "] = " << expressions[i] << ")";
      } else if (i > j) {
        EXPECT_PRED2(ExprLess, expressions[j], expressions[i])
            << "(Expressions[" << j << "] = " << expressions[j] << ")"
            << " is not less than "
            << "(Expressions[" << i << "] = " << expressions[i] << ")";
        EXPECT_PRED2(ExprNotLess, expressions[i], expressions[j])
            << "(Expressions[" << i << "] = " << expressions[i] << ")"
            << " is less than "
            << "(Expressions[" << j << "] = " << expressions[j] << ")";
      } else {
        EXPECT_PRED2(ExprNotLess, expressions[i], expressions[j])
            << "(Expressions[" << i << "] = " << expressions[i] << ")"
            << " is less than "
            << "(Expressions[" << j << "] = " << expressions[j] << ")";
        EXPECT_PRED2(ExprNotLess, expressions[j], expressions[i])
            << "(Expressions[" << j << "] = " << expressions[j] << ")"
            << " is less than "
            << "(Expressions[" << i << "] = " << expressions[i] << ")";
      }
    }
  }
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

  const Expression e_constant_{1.0};
  const Expression e_var_{var_x_};
  const Expression e_add_{x_ + y_};
  const Expression e_neg_{-x_};  // -1 * x_
  const Expression e_mul_{x_ * y_};
  const Expression e_div_{x_ / y_};
  const Expression e_log_{log(x_)};
  const Expression e_abs_{abs(x_)};
  const Expression e_exp_{exp(x_)};
  const Expression e_sqrt_{sqrt(x_)};
  const Expression e_pow_{pow(x_, y_)};
  const Expression e_sin_{sin(x_)};
  const Expression e_cos_{cos(x_)};
  const Expression e_tan_{tan(x_)};
  const Expression e_asin_{asin(x_)};
  const Expression e_acos_{acos(x_)};
  const Expression e_atan_{atan(x_)};
  const Expression e_atan2_{atan2(x_, y_)};
  const Expression e_sinh_{sinh(x_)};
  const Expression e_cosh_{cosh(x_)};
  const Expression e_tanh_{tanh(x_)};
  const Expression e_min_{min(x_, y_)};
  const Expression e_max_{max(x_, y_)};
  const Expression e_ite_{if_then_else(x_ < y_, x_, y_)};

  const vector<Expression> collection_{
      e_constant_, e_var_,  e_add_,   e_neg_,  e_mul_,
      e_div_,      e_log_,  e_abs_,   e_exp_,  e_sqrt_,
      e_pow_,      e_sin_,  e_cos_,   e_tan_,  e_asin_,
      e_acos_,     e_atan_, e_atan2_, e_sinh_, e_cosh_,
      e_tanh_,     e_min_,  e_max_,   e_ite_,  Expression::NaN()};
};

TEST_F(SymbolicExpressionTest, Dummy) {
  EXPECT_TRUE(is_nan(dummy_value<Expression>::get()));
}

TEST_F(SymbolicExpressionTest, IsConstant1) {
  EXPECT_TRUE(is_constant(e_constant_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_constant(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsConstant2) {
  EXPECT_TRUE(is_constant(Expression{}, 0.0));
  EXPECT_TRUE(is_constant(Expression::Zero(), 0.0));
  EXPECT_TRUE(is_constant(Expression::One(), 1.0));
  EXPECT_TRUE(is_constant(Expression::Pi(), M_PI));
  EXPECT_TRUE(is_constant(Expression::E(), M_E));
}

TEST_F(SymbolicExpressionTest, IsZero) {
  EXPECT_TRUE(is_zero(Expression{}));
  EXPECT_TRUE(is_zero(Expression::Zero()));
}
TEST_F(SymbolicExpressionTest, IsOne) {
  EXPECT_TRUE(is_one(Expression::One()));
}
TEST_F(SymbolicExpressionTest, IsNegOne) { EXPECT_TRUE(is_neg_one(neg_one_)); }
TEST_F(SymbolicExpressionTest, IsTwo) { EXPECT_TRUE(is_two(two_)); }

TEST_F(SymbolicExpressionTest, NaN) {
  // It's OK to have NaN expression.
  const Expression nan{NAN};
  EXPECT_TRUE(is_nan(nan));
  EXPECT_TRUE(nan.EqualTo(Expression::NaN()));
  // It's OK to have an expression including NaN inside.
  const Expression e1{1.0 + nan};
  // It's OK to display an expression including NaN inside.
  EXPECT_EQ(e1.to_string(), "(1 + NaN)");
  // It throws when we evaluate an expression including NaN.
  EXPECT_THROW(e1.Evaluate(), runtime_error);
}

TEST_F(SymbolicExpressionTest, IsVariable) {
  EXPECT_TRUE(is_variable(e_var_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_variable(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAddition) {
  EXPECT_TRUE(is_addition(e_add_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_addition(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsMultiplication) {
  EXPECT_TRUE(is_multiplication(e_mul_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_multiplication(e); })};
  EXPECT_EQ(cnt, 2);
}

TEST_F(SymbolicExpressionTest, IsDivision) {
  EXPECT_TRUE(is_division(e_div_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_division(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsLog) {
  EXPECT_TRUE(is_log(e_log_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_log(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAbs) {
  EXPECT_TRUE(is_abs(e_abs_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_abs(e); })};
  EXPECT_EQ(cnt, 1);
}
TEST_F(SymbolicExpressionTest, IsExp) {
  EXPECT_TRUE(is_exp(e_exp_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_exp(e); })};
  EXPECT_EQ(cnt, 1);
}
TEST_F(SymbolicExpressionTest, IsSqrt) {
  EXPECT_TRUE(is_sqrt(e_sqrt_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_sqrt(e); })};
  EXPECT_EQ(cnt, 1);
}
TEST_F(SymbolicExpressionTest, IsPow) {
  EXPECT_TRUE(is_pow(e_pow_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_pow(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsSin) {
  EXPECT_TRUE(is_sin(e_sin_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_sin(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsCos) {
  EXPECT_TRUE(is_cos(e_cos_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_cos(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsTan) {
  EXPECT_TRUE(is_tan(e_tan_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_tan(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAsin) {
  EXPECT_TRUE(is_asin(e_asin_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_asin(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAcos) {
  EXPECT_TRUE(is_acos(e_acos_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_acos(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAtan) {
  EXPECT_TRUE(is_atan(e_atan_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_atan(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsAtan2) {
  EXPECT_TRUE(is_atan2(e_atan2_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_atan2(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsSinh) {
  EXPECT_TRUE(is_sinh(e_sinh_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_sinh(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsCosh) {
  EXPECT_TRUE(is_cosh(e_cosh_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_cosh(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsTanh) {
  EXPECT_TRUE(is_tanh(e_tanh_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_tanh(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsMin) {
  EXPECT_TRUE(is_min(e_min_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_min(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsMax) {
  EXPECT_TRUE(is_max(e_max_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_max(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsIfThenElse) {
  EXPECT_TRUE(is_if_then_else(e_ite_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_if_then_else(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, GetConstantValue) {
  EXPECT_EQ(get_constant_value(c1_), -10.0);
  EXPECT_EQ(get_constant_value(c2_), 1.0);
  EXPECT_EQ(get_constant_value(c3_), 3.14159);
  EXPECT_EQ(get_constant_value(c4_), -2.718);
}

TEST_F(SymbolicExpressionTest, GetVariable) {
  EXPECT_EQ(get_variable(x_), var_x_);
  EXPECT_EQ(get_variable(y_), var_y_);
  EXPECT_EQ(get_variable(z_), var_z_);
}

TEST_F(SymbolicExpressionTest, GetArgument) {
  EXPECT_PRED2(ExprEqual, get_argument(e_log_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_abs_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_exp_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_sqrt_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_sin_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_cos_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_tan_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_asin_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_acos_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_atan_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_sinh_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_cosh_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_tanh_), x_);
}

TEST_F(SymbolicExpressionTest, GetFirstArgument) {
  EXPECT_PRED2(ExprEqual, get_first_argument(e_div_), x_);
  EXPECT_PRED2(ExprEqual, get_first_argument(e_pow_), x_);
  EXPECT_PRED2(ExprEqual, get_first_argument(e_atan2_), x_);
  EXPECT_PRED2(ExprEqual, get_first_argument(e_min_), x_);
  EXPECT_PRED2(ExprEqual, get_first_argument(e_max_), x_);
}

TEST_F(SymbolicExpressionTest, GetSecondArgument) {
  EXPECT_PRED2(ExprEqual, get_second_argument(e_div_), y_);
  EXPECT_PRED2(ExprEqual, get_second_argument(e_pow_), y_);
  EXPECT_PRED2(ExprEqual, get_second_argument(e_atan2_), y_);
  EXPECT_PRED2(ExprEqual, get_second_argument(e_min_), y_);
  EXPECT_PRED2(ExprEqual, get_second_argument(e_max_), y_);
}

TEST_F(SymbolicExpressionTest, GetConstantTermInAddition) {
  EXPECT_PRED2(ExprEqual, get_constant_in_addition(2 * x_ + 3 * y_), 0.0);
  EXPECT_PRED2(ExprEqual, get_constant_in_addition(3 + 2 * x_ + 3 * y_), 3);
  EXPECT_PRED2(ExprEqual, get_constant_in_addition(-2 + 2 * x_ + 3 * y_), -2);
}

TEST_F(SymbolicExpressionTest, GetTermsInAddition) {
  const Expression e{3 + 2 * x_ + 3 * y_};
  const map<Expression, double> terms{get_expr_to_coeff_map_in_addition(e)};
  EXPECT_EQ(terms.at(x_), 2.0);
  EXPECT_EQ(terms.at(y_), 3.0);
}

TEST_F(SymbolicExpressionTest, GetConstantFactorInMultiplication) {
  EXPECT_PRED2(ExprEqual, get_constant_in_multiplication(e_neg_), -1.0);
  EXPECT_PRED2(ExprEqual, get_constant_in_multiplication(x_ * y_ * y_), 1.0);
  EXPECT_PRED2(ExprEqual, get_constant_in_multiplication(2 * x_ * y_ * y_),
               2.0);
  EXPECT_PRED2(ExprEqual, get_constant_in_multiplication(-3 * x_ * y_ * y_),
               -3.0);
}

TEST_F(SymbolicExpressionTest, GetProductsInMultiplication) {
  const Expression e{2 * x_ * y_ * y_ * pow(z_, y_)};
  const map<Expression, Expression> products{
      get_base_to_exponent_map_in_multiplication(e)};
  EXPECT_PRED2(ExprEqual, products.at(x_), 1.0);
  EXPECT_PRED2(ExprEqual, products.at(y_), 2.0);
  EXPECT_PRED2(ExprEqual, products.at(z_), y_);
}

TEST_F(SymbolicExpressionTest, IsPolynomial) {
  const vector<pair<Expression, bool>> test_vec{{e_constant_, true},
                                                {e_var_, true},
                                                {e_neg_, true},
                                                {e_add_, true},
                                                {e_mul_, true},
                                                {e_div_, false},
                                                {e_log_, false},
                                                {e_abs_, false},
                                                {e_exp_, false},
                                                {e_sqrt_, false},
                                                {e_pow_, false},
                                                {e_sin_, false},
                                                {e_cos_, false},
                                                {e_tan_, false},
                                                {e_asin_, false},
                                                {e_acos_, false},
                                                {e_atan_, false},
                                                {e_atan2_, false},
                                                {e_sinh_, false},
                                                {e_cosh_, false},
                                                {e_tanh_, false},
                                                {e_min_, false},
                                                {e_max_, false},
                                                {e_ite_, false},
                                                {Expression::NaN(), false}};
  for (const pair<Expression, bool>& p : test_vec) {
    EXPECT_EQ(p.first.is_polynomial(), p.second);
  }

  // x^2 -> polynomial
  EXPECT_TRUE(pow(x_, 2).is_polynomial());
  // 3 + x + y + z -> polynomial
  EXPECT_TRUE((3 + x_ + y_ + z_).is_polynomial());
  // 1 + x^2 + y^2 -> polynomial
  EXPECT_TRUE((1 + pow(x_, 2) + pow(y_, 2)).is_polynomial());
  // x^2 * y^2 -> polynomial
  EXPECT_TRUE((pow(x_, 2) * pow(y_, 2)).is_polynomial());
  // (x + y + z)^3 -> polynomial
  EXPECT_TRUE(pow(x_ + y_ + z_, 3).is_polynomial());
  // (x + y + z)^3 / 10 -> polynomial
  EXPECT_TRUE((pow(x_ + y_ + z_, 3) / 10).is_polynomial());
  // (x^3)^(1/3) -> x -> polynomial
  EXPECT_TRUE(pow(pow(x_, 3), 1 / 3).is_polynomial());

  // x^-1 -> not polynomial
  EXPECT_FALSE(pow(x_, -1).is_polynomial());
  // x^2.1 -> not polynomial
  EXPECT_FALSE(pow(x_, 2.1).is_polynomial());
  // x^y -> not polynomial
  EXPECT_FALSE(pow(x_, y_).is_polynomial());
  // 3 + x^y -> not polynomial
  EXPECT_FALSE((3 + pow(x_, y_)).is_polynomial());
  // 3 + x^2.1 -> not polynomial
  EXPECT_FALSE((3 + pow(x_, 2.1)).is_polynomial());
  // x^y / 10 -> not polynomial
  EXPECT_FALSE((pow(x_, y_) / 10).is_polynomial());
  // x^2 * y^ 2.1 -> not polynomial
  EXPECT_FALSE((pow(x_, 2) * pow(y_, 2.1)).is_polynomial());
  // x^2 * y^ -1 -> not polynomial
  EXPECT_FALSE((pow(x_, 2) * pow(y_, -1)).is_polynomial());
  // x^2 * y^ 2 * x^y / 10 -> not polynomial
  EXPECT_FALSE((pow(x_, 2) * pow(y_, 2) * pow(x_, y_) / 10).is_polynomial());
  // (x + y + z)^3 / x -> not polynomial
  EXPECT_FALSE((pow(x_ + y_ + z_, 3) / x_).is_polynomial());
  // sqrt(x^2) -> |x| -> not polynomial
  EXPECT_FALSE(sqrt(pow(x_, 2)).is_polynomial());
}

TEST_F(SymbolicExpressionTest, ToPolynomial1) {
  Environment env{{var_x_, 1.0}, {var_y_, 2.0}, {var_z_, 3.0}};
  const map<Polynomial<double>::VarType, double> eval_point{
      {var_x_.get_id(), env[var_x_]},
      {var_y_.get_id(), env[var_y_]},
      {var_z_.get_id(), env[var_z_]}};

  const Expression e0{42.0};
  const Expression e1{pow(x_, 2)};
  const Expression e2{3 + x_ + y_ + z_};
  const Expression e3{1 + pow(x_, 2) + pow(y_, 2)};
  const Expression e4{pow(x_, 2) * pow(y_, 2)};
  const Expression e5{pow(x_ + y_ + z_, 3)};
  const Expression e6{pow(x_ + y_ + z_, 3) / 10};
  const Expression e7{-pow(y_, 3)};
  const Expression e8{pow(pow(x_, 3), 1.0 / 3)};

  EXPECT_NEAR(e0.Evaluate(env),
              e0.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e1.Evaluate(env),
              e1.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e2.Evaluate(env),
              e2.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e3.Evaluate(env),
              e3.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e4.Evaluate(env),
              e4.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e5.Evaluate(env),
              e5.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e6.Evaluate(env),
              e6.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e7.Evaluate(env),
              e7.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
  EXPECT_NEAR(e8.Evaluate(env),
              e8.ToPolynomial().EvaluateMultivariate(eval_point), 1e-8);
}

TEST_F(SymbolicExpressionTest, ToPolynomial2) {
  const vector<Expression> test_vec{
      e_log_,  e_abs_,  e_exp_,  e_sqrt_, e_sin_,   e_cos_,
      e_tan_,  e_asin_, e_acos_, e_atan_, e_atan2_, e_sinh_,
      e_cosh_, e_tanh_, e_min_,  e_max_,  e_ite_,   Expression::NaN()};
  for (const Expression& e : test_vec) {
    EXPECT_FALSE(e.is_polynomial());
    EXPECT_THROW(e.ToPolynomial(), runtime_error);
  }
}

TEST_F(SymbolicExpressionTest, LessKind) {
  CheckOrdering({e_constant_, e_var_,  e_add_,   e_neg_,  e_mul_,
                 e_div_,      e_log_,  e_abs_,   e_exp_,  e_sqrt_,
                 e_pow_,      e_sin_,  e_cos_,   e_tan_,  e_asin_,
                 e_acos_,     e_atan_, e_atan2_, e_sinh_, e_cosh_,
                 e_tanh_,     e_min_,  e_max_,   e_ite_,  Expression::NaN()});
}

TEST_F(SymbolicExpressionTest, LessConstant) { CheckOrdering({c1_, c2_, c3_}); }

TEST_F(SymbolicExpressionTest, LessVariable) { CheckOrdering({x_, y_, z_}); }

TEST_F(SymbolicExpressionTest, LessNeg) {
  // Defined in the ascending order.
  const Expression neg1{-c3_};
  const Expression neg2{-c1_};
  const Expression neg3{-x_};  // note: Constant kind < Variable kind
  CheckOrdering({neg1, neg2, neg3});
}

TEST_F(SymbolicExpressionTest, LessAdd) {
  const Expression add1{c1_ + x_ + y_};
  const Expression add2{c1_ + 2 * x_ + y_};
  const Expression add3{c1_ - 2 * y_ + z_};
  const Expression add4{c1_ + y_ + z_};
  const Expression add5{c1_ + 5 * y_ + z_};
  const Expression add6{c3_ - 2 * x_ + y_};
  const Expression add7{c3_ + x_ + y_};
  const Expression add8{c3_ + y_ + 2 * z_};
  const Expression add9{c3_ + y_ + 3 * z_};
  CheckOrdering({add1, add2, add3, add4, add5, add6, add7, add8, add9});
}

TEST_F(SymbolicExpressionTest, LessSub) {
  const Expression sub1{c1_ - x_ - y_};
  const Expression sub2{c1_ - y_ - z_};
  const Expression sub3{c3_ - x_ - y_};
  const Expression sub4{c3_ - y_ - z_};
  CheckOrdering({sub1, sub2, sub3, sub4});
}

TEST_F(SymbolicExpressionTest, LessMul) {
  const Expression mul1{c1_ * x_ * y_};
  const Expression mul2{c1_ * y_ * z_};
  const Expression mul3{c3_ * x_ * y_};
  const Expression mul4{c3_ * y_ * z_};
  CheckOrdering({mul1, mul2, mul3, mul4});
}

TEST_F(SymbolicExpressionTest, LessDiv) {
  const Expression div1{x_ / y_};
  const Expression div2{x_ / z_};
  const Expression div3{y_ / z_};
  CheckOrdering({div1, div2, div3});
}

TEST_F(SymbolicExpressionTest, LessLog) {
  const Expression log1{log(x_)};
  const Expression log2{log(y_)};
  const Expression log3{log(x_plus_y_)};
  const Expression log4{log(x_plus_z_)};
  CheckOrdering({log1, log2, log3, log4});
}

TEST_F(SymbolicExpressionTest, LessAbs) {
  const Expression abs1{abs(x_)};
  const Expression abs2{abs(y_)};
  const Expression abs3{abs(x_plus_y_)};
  const Expression abs4{abs(x_plus_z_)};
  CheckOrdering({abs1, abs2, abs3, abs4});
}

TEST_F(SymbolicExpressionTest, LessExp) {
  const Expression exp1{exp(x_)};
  const Expression exp2{exp(y_)};
  const Expression exp3{exp(x_plus_y_)};
  const Expression exp4{exp(x_plus_z_)};
  CheckOrdering({exp1, exp2, exp3, exp4});
}

TEST_F(SymbolicExpressionTest, LessSqrt) {
  const Expression sqrt1{sqrt(x_)};
  const Expression sqrt2{sqrt(y_)};
  const Expression sqrt3{sqrt(x_plus_y_)};
  const Expression sqrt4{sqrt(x_plus_z_)};
  CheckOrdering({sqrt1, sqrt2, sqrt3, sqrt4});
}

TEST_F(SymbolicExpressionTest, LessSin) {
  const Expression sin1{sin(x_)};
  const Expression sin2{sin(y_)};
  const Expression sin3{sin(x_plus_y_)};
  const Expression sin4{sin(x_plus_z_)};
  CheckOrdering({sin1, sin2, sin3, sin4});
}

TEST_F(SymbolicExpressionTest, LessCos) {
  const Expression cos1{cos(x_)};
  const Expression cos2{cos(y_)};
  const Expression cos3{cos(x_plus_y_)};
  const Expression cos4{cos(x_plus_z_)};
  CheckOrdering({cos1, cos2, cos3, cos4});
}

TEST_F(SymbolicExpressionTest, LessTan) {
  const Expression tan1{tan(x_)};
  const Expression tan2{tan(y_)};
  const Expression tan3{tan(x_plus_y_)};
  const Expression tan4{tan(x_plus_z_)};
  CheckOrdering({tan1, tan2, tan3, tan4});
}

TEST_F(SymbolicExpressionTest, LessAsin) {
  const Expression asin1{asin(x_)};
  const Expression asin2{asin(y_)};
  const Expression asin3{asin(x_plus_y_)};
  const Expression asin4{asin(x_plus_z_)};
  CheckOrdering({asin1, asin2, asin3, asin4});
}

TEST_F(SymbolicExpressionTest, LessAcos) {
  const Expression acos1{acos(x_)};
  const Expression acos2{acos(y_)};
  const Expression acos3{acos(x_plus_y_)};
  const Expression acos4{acos(x_plus_z_)};
  CheckOrdering({acos1, acos2, acos3, acos4});
}

TEST_F(SymbolicExpressionTest, LessAtan) {
  const Expression atan1{atan(x_)};
  const Expression atan2{atan(y_)};
  const Expression atan3{atan(x_plus_y_)};
  const Expression atan4{atan(x_plus_z_)};
  CheckOrdering({atan1, atan2, atan3, atan4});
}

TEST_F(SymbolicExpressionTest, LessSinh) {
  const Expression sinh1{sinh(x_)};
  const Expression sinh2{sinh(y_)};
  const Expression sinh3{sinh(x_plus_y_)};
  const Expression sinh4{sinh(x_plus_z_)};
  CheckOrdering({sinh1, sinh2, sinh3, sinh4});
}

TEST_F(SymbolicExpressionTest, LessCosh) {
  const Expression cosh1{cosh(x_)};
  const Expression cosh2{cosh(y_)};
  const Expression cosh3{cosh(x_plus_y_)};
  const Expression cosh4{cosh(x_plus_z_)};
  CheckOrdering({cosh1, cosh2, cosh3, cosh4});
}

TEST_F(SymbolicExpressionTest, LessTanh) {
  const Expression tanh1{tanh(x_)};
  const Expression tanh2{tanh(y_)};
  const Expression tanh3{tanh(x_plus_y_)};
  const Expression tanh4{tanh(x_plus_z_)};
  CheckOrdering({tanh1, tanh2, tanh3, tanh4});
}

TEST_F(SymbolicExpressionTest, LessPow) {
  const Expression pow1{pow(x_, y_)};
  const Expression pow2{pow(x_, z_)};
  const Expression pow3{pow(y_, z_)};
  CheckOrdering({pow1, pow2, pow3});
}

TEST_F(SymbolicExpressionTest, LessAtan2) {
  const Expression atan2_1{atan2(x_, y_)};
  const Expression atan2_2{atan2(x_, z_)};
  const Expression atan2_3{atan2(y_, z_)};
  CheckOrdering({atan2_1, atan2_2, atan2_3});
}

TEST_F(SymbolicExpressionTest, LessMin) {
  const Expression min1{min(x_, y_)};
  const Expression min2{min(x_, z_)};
  const Expression min3{min(y_, z_)};
  CheckOrdering({min1, min2, min3});
}

TEST_F(SymbolicExpressionTest, LessMax) {
  const Expression max1{max(x_, y_)};
  const Expression max2{max(x_, z_)};
  const Expression max3{max(y_, z_)};
  CheckOrdering({max1, max2, max3});
}

TEST_F(SymbolicExpressionTest, LessIfThenElse) {
  const Formula f1{x_ < y_};
  const Formula f2{y_ < z_};
  const Expression ite1{if_then_else(f1, x_, y_)};
  const Expression ite2{if_then_else(f1, x_, z_)};
  const Expression ite3{if_then_else(f1, y_, z_)};
  const Expression ite4{if_then_else(f2, y_, z_)};
  const Expression ite5{if_then_else(f2, z_, x_)};
  CheckOrdering({ite1, ite2, ite3, ite4, ite5});
}

TEST_F(SymbolicExpressionTest, Variable) {
  EXPECT_EQ(x_.to_string(), var_x_.get_name());
  EXPECT_EQ(y_.to_string(), var_y_.get_name());
  EXPECT_EQ(z_.to_string(), var_z_.get_name());
  EXPECT_PRED2(ExprEqual, x_, x_);
  EXPECT_PRED2(ExprNotEqual, x_, y_);
  EXPECT_PRED2(ExprNotEqual, x_, z_);
  EXPECT_PRED2(ExprNotEqual, y_, x_);
  EXPECT_PRED2(ExprEqual, y_, y_);
  EXPECT_PRED2(ExprNotEqual, y_, z_);
  EXPECT_PRED2(ExprNotEqual, z_, x_);
  EXPECT_PRED2(ExprNotEqual, z_, y_);
  EXPECT_PRED2(ExprEqual, z_, z_);
}

TEST_F(SymbolicExpressionTest, Evaluate) {
  EXPECT_THROW(x_plus_y_.Evaluate(), std::runtime_error);
}

TEST_F(SymbolicExpressionTest, Constant) {
  EXPECT_EQ(c1_.Evaluate(), -10);
  EXPECT_EQ(c2_.Evaluate(), 1);
  EXPECT_EQ(c3_.Evaluate(), 3.14159);
  EXPECT_EQ(c4_.Evaluate(), -2.718);
  EXPECT_THROW(Expression{NAN}.Evaluate(), runtime_error);
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
  EXPECT_PRED2(ExprNotEqual, c3_, -c3_);
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), -(-c3_).Evaluate());
  EXPECT_PRED2(ExprEqual, c3_, -(-c3_));
  EXPECT_DOUBLE_EQ(c3_.Evaluate(), (-(-c3_)).Evaluate());
  const Expression e{x_ + y_};
  const Environment env{{var_x_, 1.0}, {var_y_, 2.0}};
  EXPECT_EQ((-x_).Evaluate(env), -1.0);
  EXPECT_PRED2(ExprEqual, x_, -(-x_));
  // (x + y) and -(-(x + y)) are structurally equal (after simplification)
  EXPECT_PRED2(ExprEqual, e, -(-e));
  // and their evaluations should be the same.
  EXPECT_DOUBLE_EQ(e.Evaluate(env), (-(-e)).Evaluate(env));

  EXPECT_PRED2(ExprEqual, -(x_plus_y_ + x_plus_z_ + pi_),
               -x_plus_y_ + (-x_plus_z_) + (-pi_));
  EXPECT_EQ((-(x_)).to_string(), "(-1 * x)");
}

TEST_F(SymbolicExpressionTest, Add1) {
  EXPECT_PRED2(ExprEqual, c3_ + zero_, c3_);
  EXPECT_EQ((c3_ + zero_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, zero_ + c3_, c3_);
  EXPECT_EQ((zero_ + c3_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, 0.0 + c3_, c3_);
  EXPECT_EQ((0.0 + c3_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, c3_ + 0.0, c3_);
  EXPECT_EQ((c3_ + 0.0).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, c3_ + c4_, 3.14159 + -2.718);
  EXPECT_EQ((c3_ + c4_).to_string(), Expression{3.14159 + -2.718}.to_string());
  EXPECT_PRED2(ExprEqual, c3_ + x_, 3.14159 + x_);
  EXPECT_EQ((c3_ + x_).to_string(), (3.14159 + x_).to_string());
  EXPECT_PRED2(ExprEqual, x_ + c3_, x_ + 3.14159);
  EXPECT_EQ((x_ + c3_).to_string(), (x_ + 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Add2) {
  Expression e1{x_ + y_};
  Expression e2{e1 + e1};
  const auto str_rep_e2(e2.to_string());
  EXPECT_EQ(str_rep_e2, "(2 * x + 2 * y)");
  EXPECT_PRED2(ExprEqual, e2, 2 * x_ + 2 * y_);
  e1 += z_;
  EXPECT_PRED2(ExprEqual, e1, x_ + y_ + z_);
  EXPECT_EQ(e2.to_string(), str_rep_e2);  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Add3) {
  const Expression e1{2 + x_ + y_};
  const Expression e2{3 + x_ + y_};
  EXPECT_PRED2(ExprNotEqual, e1, e2);
}

TEST_F(SymbolicExpressionTest, Add4) {
  const Expression e1{-2 - x_ + -3 * y_};
  const Expression e2{-2 - x_ - 3 * y_};
  EXPECT_EQ(e1.to_string(), "(-2 - x - 3 * y)");
  EXPECT_EQ(e2.to_string(), "(-2 - x - 3 * y)");
}

TEST_F(SymbolicExpressionTest, Inc1) {
  // Prefix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};
  EXPECT_PRED2(ExprEqual, x, x_prime);
  EXPECT_PRED2(ExprEqual, x++, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime++);
  EXPECT_PRED2(ExprEqual, x, x_prime);
}

TEST_F(SymbolicExpressionTest, Inc2) {
  // Postfix increment
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExprEqual, x, x_prime);
  EXPECT_PRED2(ExprNotEqual, ++x, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime);
  EXPECT_PRED2(ExprEqual, x, ++x_prime);
  EXPECT_PRED2(ExprEqual, x, x_prime);
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
  EXPECT_PRED2(ExprEqual, c3_ - zero_, c3_);
  EXPECT_EQ((c3_ - zero_).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, zero_ - c3_, -c3_);
  EXPECT_EQ((zero_ - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_PRED2(ExprEqual, 0.0 - c3_, -c3_);
  EXPECT_EQ((0.0 - c3_).to_string(), Expression{-3.14159}.to_string());
  EXPECT_PRED2(ExprEqual, 0.0 - c3_, (-1 * c3_));
  EXPECT_EQ((0.0 - c3_).to_string(), (-1 * c3_).to_string());
  EXPECT_PRED2(ExprEqual, c3_ - 0.0, c3_);
  EXPECT_EQ((c3_ - 0.0).to_string(), c3_.to_string());
  EXPECT_PRED2(ExprEqual, c3_ - c4_, Expression{3.14159 - -2.718});
  EXPECT_EQ((c3_ - c4_).to_string(), Expression{3.14159 - -2.718}.to_string());
  EXPECT_PRED2(ExprEqual, c3_ - x_, 3.14159 - x_);
  EXPECT_EQ((c3_ - x_).to_string(), (3.14159 - x_).to_string());
  EXPECT_PRED2(ExprEqual, x_ - c3_, x_ - 3.14159);
  EXPECT_EQ((x_ - c3_).to_string(), (x_ - 3.14159).to_string());
}

TEST_F(SymbolicExpressionTest, Sub2) {
  Expression e1{x_ - y_};
  const Expression e2{x_ - z_};
  const Expression e3{e1 - e2};
  const auto str_rep_e3(e3.to_string());
  EXPECT_EQ(str_rep_e3, "( - y + z)");
  e1 -= z_;
  EXPECT_PRED2(ExprEqual, e1, x_ - y_ - z_);
  EXPECT_EQ(e3.to_string(), str_rep_e3);  // e3 doesn't change.
}

TEST_F(SymbolicExpressionTest, Sub3) {
  const Expression e1{x_ - y_};
  const Expression e2{x_ - y_};
  const Expression e3{e1 - e2};
  EXPECT_PRED2(ExprEqual, e1, e2);
  EXPECT_EQ(e3.to_string(), "0");  // simplified
}

TEST_F(SymbolicExpressionTest, Dec1) {
  // Postfix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExprEqual, x, x_prime);
  EXPECT_PRED2(ExprEqual, x--, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime--);
  EXPECT_PRED2(ExprEqual, x, x_prime);
}

TEST_F(SymbolicExpressionTest, Dec2) {
  // Prefix decrement.
  Expression x{var_x_};
  Expression x_prime{var_x_};

  EXPECT_PRED2(ExprEqual, x, x_prime);
  EXPECT_PRED2(ExprNotEqual, --x, x_prime);
  EXPECT_PRED2(ExprNotEqual, x, x_prime);
  EXPECT_PRED2(ExprEqual, x, --x_prime);
  EXPECT_PRED2(ExprEqual, x, x_prime);
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
  EXPECT_PRED2(ExprEqual, c3_ * zero_, zero_);
  EXPECT_PRED2(ExprEqual, zero_ * c3_, zero_);
  EXPECT_PRED2(ExprEqual, c3_ * 0.0, zero_);
  EXPECT_PRED2(ExprEqual, 0.0 * c3_, zero_);

  EXPECT_PRED2(ExprEqual, c3_ * one_, c3_);
  EXPECT_PRED2(ExprEqual, one_ * c3_, c3_);
  EXPECT_PRED2(ExprEqual, 1.0 * c3_, c3_);
  EXPECT_PRED2(ExprEqual, c3_ * 1.0, c3_);

  EXPECT_PRED2(ExprEqual, c3_ * c4_, 3.14159 * -2.718);
  EXPECT_PRED2(ExprEqual, c3_ * x_, (3.14159 * x_));
  EXPECT_PRED2(ExprEqual, x_ * c3_, (x_ * 3.14159));
}

TEST_F(SymbolicExpressionTest, Mul2) {
  Expression e1{x_ * y_};
  Expression e2{e1 * e1};
  EXPECT_EQ(e1.to_string(), "(x * y)");
  EXPECT_PRED2(ExprEqual, e2, pow(x_, 2) * pow(y_, 2));
  e1 *= z_;
  EXPECT_PRED2(ExprEqual, e1, x_ * y_ * z_);
  EXPECT_PRED2(ExprEqual, e2, pow(x_, 2) * pow(y_, 2));  // e2 doesn't change.
}

TEST_F(SymbolicExpressionTest, Mul3) {
  const Expression e1{x_ * x_ * x_ * x_};
  const Expression e2{(x_ * x_) * (x_ * x_)};
  const Expression e3{x_ * (x_ * x_ * x_)};
  const Expression e4{pow(x_, 4)};
  EXPECT_PRED2(ExprEqual, e1, e4);
  EXPECT_PRED2(ExprEqual, e2, e4);
  EXPECT_PRED2(ExprEqual, e3, e4);
}

TEST_F(SymbolicExpressionTest, Mul4) {
  // x * y * y * x = x^2 * y^2
  EXPECT_PRED2(ExprEqual, x_ * y_ * y_ * x_, pow(x_, 2) * pow(y_, 2));
  // (x * y * y * x) * (x * y * x * y) = x^4 * y^4
  EXPECT_PRED2(ExprEqual, (x_ * y_ * y_ * x_) * (x_ * y_ * x_ * y_),
               pow(x_, 4) * pow(y_, 4));
  EXPECT_PRED2(ExprEqual, 3 * (4 * x_), (3 * 4) * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * 4, (3 * 4) * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * (4 * x_), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExprEqual, (x_ * 3) * (x_ * 4), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * (4 * y_), (3 * 4) * x_ * y_);
}

TEST_F(SymbolicExpressionTest, Mul5) {
  // x^y * x^z = x^(y + z)
  EXPECT_PRED2(ExprEqual, pow(x_, y_) * pow(x_, z_), pow(x_, y_ + z_));

  // x^y * x = x^(y + 1)
  EXPECT_PRED2(ExprEqual, pow(x_, y_) * x_, pow(x_, y_ + 1));

  // x * pow(x, y) = x^(1 + y)
  EXPECT_PRED2(ExprEqual, x_ * pow(x_, y_), pow(x_, 1 + y_));

  // x * y * y * x = x^2 * y^2
  EXPECT_PRED2(ExprEqual, x_ * y_ * y_ * x_, pow(x_, 2) * pow(y_, 2));
  // (x * y * y * x) * (x * y * x * y) = x^4 * y^4
  EXPECT_PRED2(ExprEqual, (x_ * y_ * y_ * x_) * (x_ * y_ * x_ * y_),
               pow(x_, 4) * pow(y_, 4));
  EXPECT_PRED2(ExprEqual, 3 * (4 * x_), (3 * 4) * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * 4, (3 * 4) * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * (4 * x_), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExprEqual, (x_ * 3) * (x_ * 4), (3 * 4) * x_ * x_);
  EXPECT_PRED2(ExprEqual, (3 * x_) * (4 * y_), (3 * 4) * x_ * y_);
}

TEST_F(SymbolicExpressionTest, Mul6) {
  EXPECT_EQ((x_ * x_ * y_ * y_ * y_).to_string(), "(pow(x, 2) * pow(y, 3))");
  EXPECT_EQ((2 * x_ * x_ * y_ * y_ * y_).to_string(),
            "(2 * pow(x, 2) * pow(y, 3))");
  EXPECT_EQ((-3 * x_ * x_ * y_ * y_ * y_).to_string(),
            "(-3 * pow(x, 2) * pow(y, 3))");
}

TEST_F(SymbolicExpressionTest, Mul7) {
  const Expression e1{2 * pow(x_, 2)};
  const Expression e2{3 * pow(x_, -2)};
  EXPECT_PRED2(ExprEqual, e1 * e2, 6);
}

TEST_F(SymbolicExpressionTest, AddMul1) {
  const Expression e1{(x_ * y_ * y_ * x_) + (x_ * y_ * x_ * y_)};
  EXPECT_PRED2(ExprEqual, e1, 2 * pow(x_, 2) * pow(y_, 2));

  const Expression e2{x_ + y_ + (-x_)};
  EXPECT_PRED2(ExprEqual, e2, y_);

  const Expression e3{(2 * x_) + (3 * x_)};
  EXPECT_PRED2(ExprEqual, e3, 5 * x_);

  const Expression e4{(x_ * 2 * x_) + (x_ * x_ * 3)};
  EXPECT_PRED2(ExprEqual, e4, 5 * x_ * x_);
  EXPECT_PRED2(ExprEqual, e4, 5 * pow(x_, 2));
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

TEST_F(SymbolicExpressionTest, Div4) {
  const Expression e{x_ / y_};
  const Environment env1{{var_x_, 1.0}, {var_y_, 5.0}};
  const Environment env2{{var_x_, 1.0}, {var_y_, 0.0}};
  EXPECT_EQ(e.Evaluate(env1), 1.0 / 5.0);
  EXPECT_THROW(e.Evaluate(env2), std::runtime_error);
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
  vec.push_back(123.0);
}

GTEST_TEST(ExpressionTest, NoThrowMoveConstructible) {
  // Make sure that symbolic::Expression is nothrow move-constructible so that
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
  EXPECT_EQ((log(x_)).to_string(), "log(x)");
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
  EXPECT_EQ((abs(x_)).to_string(), "abs(x)");
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
  EXPECT_EQ((exp(x_)).to_string(), "exp(x)");
}

TEST_F(SymbolicExpressionTest, Sqrt1) {
  // sqrt(x * x) => |x|
  EXPECT_PRED2(ExprEqual, sqrt(x_plus_y_ * x_plus_y_), abs(x_plus_y_));
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
  EXPECT_EQ((sqrt(x_)).to_string(), "sqrt(x)");
}

TEST_F(SymbolicExpressionTest, Pow1) {
  // pow(x, 0.0) => 1.0
  EXPECT_PRED2(ExprEqual, pow(x_plus_y_, Expression::Zero()),
               Expression::One());
  // pow(x, 1.0) => x
  EXPECT_PRED2(ExprEqual, pow(x_plus_y_, Expression::One()), x_plus_y_);
  // (x^2)^3 => x^(2*3)
  EXPECT_PRED2(ExprEqual, pow(pow(x_, 2.0), 3.0), pow(x_, 2.0 * 3.0));
  // (x^y)^z => x^(y*z)
  EXPECT_PRED2(ExprEqual, pow(pow(x_, y_), z_), pow(x_, y_ * z_));
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

  const Expression e1{pow(x_ * y_ * pi_, x_ + y_ + pi_)};
  const Expression e2{(pow(x_, 2) * pow(y_, 2) * pow(x_, y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e1.Evaluate(env),
                   std::pow(2 * 3.2 * 3.141592, 2 + 3.2 + 3.141592));
  EXPECT_DOUBLE_EQ(e2.Evaluate(env),
                   std::pow(2, 2) * std::pow(3.2, 2) * std::pow(2, 3.2));
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
  EXPECT_EQ((sin(x_)).to_string(), "sin(x)");
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
  EXPECT_EQ((cos(x_)).to_string(), "cos(x)");
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
  EXPECT_EQ((tan(x_)).to_string(), "tan(x)");
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
  EXPECT_EQ((asin(x_)).to_string(), "asin(x)");
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
  EXPECT_EQ((acos(x_)).to_string(), "acos(x)");
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
  EXPECT_EQ((atan(x_)).to_string(), "atan(x)");
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
  EXPECT_EQ((atan2(x_, y_)).to_string(), "atan2(x, y)");
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
  EXPECT_EQ((sinh(x_)).to_string(), "sinh(x)");
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
  EXPECT_EQ((cosh(x_)).to_string(), "cosh(x)");
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
  EXPECT_EQ((tanh(x_)).to_string(), "tanh(x)");
}

TEST_F(SymbolicExpressionTest, Min1) {
  // min(E, E) -> E
  EXPECT_PRED2(ExprEqual, min(x_plus_y_, x_plus_y_), x_plus_y_);
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
  EXPECT_EQ((min(x_, y_)).to_string(), "min(x, y)");
}

TEST_F(SymbolicExpressionTest, Max1) {
  // max(E, E) -> E
  EXPECT_PRED2(ExprEqual, max(x_plus_y_, x_plus_y_), x_plus_y_);
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
  EXPECT_EQ((max(x_, y_)).to_string(), "max(x, y)");
}

TEST_F(SymbolicExpressionTest, IfThenElse1) {
  // should be simplified to 1.0 since (x + 1.0 > x) => true.
  const Expression ite1{if_then_else(x_ + 1.0 > x_, 1.0, 0.0)};
  EXPECT_PRED2(ExprEqual, ite1, 1.0);

  // should be simplified to 0.0 since (x > x + 1.0) => false.
  const Expression ite2{if_then_else(x_ > x_ + 1.0, 1.0, 0.0)};
  EXPECT_PRED2(ExprEqual, ite2, 0.0);

  // should not be simplified.
  const Expression ite3{if_then_else(x_ > y_, 1.0, 0.0)};
  EXPECT_PRED2(ExprNotEqual, ite3, 1.0);
  EXPECT_PRED2(ExprNotEqual, ite3, 0.0);
}

TEST_F(SymbolicExpressionTest, IfThenElse2) {
  const Expression max_fn{if_then_else(x_ > y_, x_, y_)};
  const Expression min_fn{if_then_else(x_ < y_, x_, y_)};

  const Environment env1{{var_x_, 5.0}, {var_y_, 3.0}};
  EXPECT_EQ(max_fn.Evaluate(env1), std::max(5.0, 3.0));
  EXPECT_EQ(min_fn.Evaluate(env1), std::min(5.0, 3.0));

  const Environment env2{{var_x_, 2.0}, {var_y_, 7.0}};
  EXPECT_EQ(max_fn.Evaluate(env2), std::max(2.0, 7.0));
  EXPECT_EQ(min_fn.Evaluate(env2), std::min(2.0, 7.0));
  EXPECT_EQ(max_fn.to_string(), "(if (x > y) then x else y)");
}

TEST_F(SymbolicExpressionTest, IfThenElse3) {
  const Expression max_fn{if_then_else(x_ > 1.0, y_, z_)};
  const Variables vars{max_fn.GetVariables()};
  EXPECT_EQ(vars.size(), 3u);
}

TEST_F(SymbolicExpressionTest, Cond1) {
  const Expression e{cond(x_ >= 10, 10, 0.0)};
  EXPECT_PRED2(ExprEqual, e, if_then_else(x_ >= 10, 10, 0.0));

  EXPECT_EQ(e.Evaluate({{var_x_, 15}}), 10.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 10}}), 10.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 0}}), 0.0);
}

TEST_F(SymbolicExpressionTest, Cond2) {
  // clang-format off
  const Expression e{cond(x_ >= 10, 10.0,
                          x_ >= 5,  5.0,
                          x_ >= 2,  2.0,
                                    0.0)};
  EXPECT_PRED2(ExprEqual, e,
               if_then_else(x_ >= 10, 10,
               if_then_else(x_ >= 5,   5,
               if_then_else(x_ >= 2,   2,
                                     0.0))));
  // clang-format on

  EXPECT_EQ(e.Evaluate({{var_x_, 15}}), 10.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 10}}), 10.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 9}}), 5.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 5}}), 5.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 3}}), 2.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 2}}), 2.0);
  EXPECT_EQ(e.Evaluate({{var_x_, 1}}), 0.0);
}

TEST_F(SymbolicExpressionTest, GetVariables) {
  const Variables vars1{(x_ + y_ * log(x_ + y_)).GetVariables()};
  EXPECT_TRUE(vars1.include(var_x_));
  EXPECT_TRUE(vars1.include(var_y_));
  EXPECT_FALSE(vars1.include(var_z_));
  EXPECT_EQ(vars1.size(), 2u);

  const Variables vars2{(x_ * x_ * z_ - y_ * abs(x_) * log(x_ + y_) + cosh(x_) +
                         cosh(y_) + atan2(x_, y_))
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
  EXPECT_PRED2(ExprEqual, e1, e1_copy);
  EXPECT_PRED2(ExprEqual, e2, e2_copy);

  swap(e1, e2);

  // After Swap.
  EXPECT_PRED2(ExprEqual, e1, e2_copy);
  EXPECT_PRED2(ExprEqual, e2, e1_copy);
}

TEST_F(SymbolicExpressionTest, ToString) {
  const Expression e1{sin(x_ + y_ * z_)};
  const Expression e2{cos(x_ * x_ + pow(y_, 2) * z_)};

  EXPECT_EQ(e1.to_string(), "sin((x + (y * z)))");
  EXPECT_EQ(e2.to_string(), "cos(((pow(y, 2) * z) + pow(x, 2)))");
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
