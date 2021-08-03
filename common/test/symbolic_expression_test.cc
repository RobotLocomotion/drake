#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <random>
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
#include "drake/common/polynomial.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/is_memcpy_movable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

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

using test::IsMemcpyMovable;

namespace symbolic {
namespace {

using test::ExprEqual;
using test::ExprLess;
using test::ExprNotEqual;
using test::ExprNotLess;
using test::FormulaEqual;

template <typename T>
size_t get_std_hash(const T& item) {
  return std::hash<T>{}(item);
}

// Checks if a given 'expressions' is ordered by Expression::Less.
void CheckOrdering(const vector<Expression>& expressions) {
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
  const Variable var_a_{"a"};
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression a_{var_a_};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Expression x_plus_y_{x_ + y_};
  const Expression x_plus_z_{x_ + z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{M_PI};
  const Expression neg_pi_{-M_PI};
  const Expression e_{M_E};

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
  const Expression e_ceil_{ceil(x_)};
  const Expression e_floor_{floor(x_)};
  const Expression e_ite_{if_then_else(x_ < y_, x_, y_)};
  const Expression e_nan_{Expression::NaN()};
  const Expression e_uf_{uninterpreted_function("uf", {var_x_, var_y_})};

  const vector<Expression> collection_{
      e_constant_, e_var_,  e_add_,  e_neg_,   e_mul_,  e_div_,  e_log_,
      e_abs_,      e_exp_,  e_sqrt_, e_pow_,   e_sin_,  e_cos_,  e_tan_,
      e_asin_,     e_acos_, e_atan_, e_atan2_, e_sinh_, e_cosh_, e_tanh_,
      e_min_,      e_max_,  e_ceil_, e_floor_, e_ite_,  e_nan_,  e_uf_};
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

TEST_F(SymbolicExpressionTest, IsCeil) {
  EXPECT_TRUE(is_ceil(e_ceil_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_ceil(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsFloor) {
  EXPECT_TRUE(is_floor(e_floor_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_floor(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsIfThenElse) {
  EXPECT_TRUE(is_if_then_else(e_ite_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_if_then_else(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsNaN) {
  EXPECT_TRUE(is_nan(e_nan_));
  const vector<Expression>::difference_type cnt{
      count_if(collection_.begin(), collection_.end(),
               [](const Expression& e) { return is_nan(e); })};
  EXPECT_EQ(cnt, 1);
}

TEST_F(SymbolicExpressionTest, IsUninterpretedFunction) {
  EXPECT_TRUE(is_uninterpreted_function(e_uf_));
  const vector<Expression>::difference_type cnt{count_if(
      collection_.begin(), collection_.end(),
      [](const Expression& e) { return is_uninterpreted_function(e); })};
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
  EXPECT_PRED2(ExprEqual, get_argument(e_ceil_), x_);
  EXPECT_PRED2(ExprEqual, get_argument(e_floor_), x_);
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

TEST_F(SymbolicExpressionTest, GetIfThenElse) {
  const Formula conditional{x_ > y_};
  const Expression e1{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e{if_then_else(conditional, e1, e2)};
  EXPECT_PRED2(FormulaEqual, get_conditional_formula(e), conditional);
  EXPECT_PRED2(ExprEqual, get_then_expression(e), e1);
  EXPECT_PRED2(ExprEqual, get_else_expression(e), e2);
}

TEST_F(SymbolicExpressionTest, IsPolynomial) {
  const vector<pair<Expression, bool>> test_vec{
      {e_constant_, true}, {e_var_, true},   {e_neg_, true},
      {e_add_, true},      {e_mul_, true},   {e_div_, false},
      {e_log_, false},     {e_abs_, false},  {e_exp_, false},
      {e_sqrt_, false},    {e_pow_, false},  {e_sin_, false},
      {e_cos_, false},     {e_tan_, false},  {e_asin_, false},
      {e_acos_, false},    {e_atan_, false}, {e_atan2_, false},
      {e_sinh_, false},    {e_cosh_, false}, {e_tanh_, false},
      {e_min_, false},     {e_max_, false},  {e_ceil_, false},
      {e_floor_, false},   {e_ite_, false},  {e_nan_, false},
      {e_uf_, false}};
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

TEST_F(SymbolicExpressionTest, LessKind) {
  CheckOrdering({e_constant_, e_var_,  e_add_,  e_neg_,  e_mul_,  e_div_,
                 e_log_,      e_abs_,  e_exp_,  e_sqrt_, e_pow_,  e_sin_,
                 e_cos_,      e_tan_,  e_asin_, e_acos_, e_atan_, e_atan2_,
                 e_sinh_,     e_cosh_, e_tanh_, e_min_,  e_max_,  e_ceil_,
                 e_floor_,    e_ite_,  e_nan_,  e_uf_});
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

TEST_F(SymbolicExpressionTest, LessCeil) {
  const Expression ceil1{ceil(x_)};
  const Expression ceil2{ceil(y_)};
  const Expression ceil3{ceil(x_plus_y_)};
  const Expression ceil4{ceil(x_plus_z_)};
  CheckOrdering({ceil1, ceil2, ceil3, ceil4});
}

TEST_F(SymbolicExpressionTest, LessFloor) {
  const Expression floor1{floor(x_)};
  const Expression floor2{floor(y_)};
  const Expression floor3{floor(x_plus_y_)};
  const Expression floor4{floor(x_plus_z_)};
  CheckOrdering({floor1, floor2, floor3, floor4});
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

TEST_F(SymbolicExpressionTest, LessUninterpretedFunction) {
  const Expression uf1{uninterpreted_function("name1", {x_, y_ + z_})};
  const Expression uf2{uninterpreted_function("name1", {x_, y_ * z_})};
  const Expression uf3{uninterpreted_function("name1", {x_, y_ * z_, 3.0})};
  const Expression uf4{uninterpreted_function("name2", {})};
  const Expression uf5{uninterpreted_function("name2", {0.0, -1.0})};
  const Expression uf6{uninterpreted_function("name2", {1.0, 0.0})};
  CheckOrdering({uf1, uf2, uf3, uf4, uf5, uf6});
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
  EXPECT_NEAR(Expression::Pi().Evaluate(), M_PI, 0.000001);
  EXPECT_NEAR(Expression::E().Evaluate(), M_E, 0.000000001);
}

TEST_F(SymbolicExpressionTest, Hash) {
  Expression x{var_x_};
  const Expression x_prime(x);
  EXPECT_EQ(get_std_hash(x), get_std_hash(x_prime));
  x++;
  EXPECT_NE(get_std_hash(x), get_std_hash(x_prime));
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
    hash_set.insert(get_std_hash(e));
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
  const Expression e13{ceil(x_plus_y_)};
  const Expression e14{floor(x_plus_y_)};

  // e0, ..., e14 share the same sub-expression, but their hash values should be
  // distinct.
  unordered_set<size_t> hash_set;
  const vector<Expression> exprs{e0, e1, e2,  e3,  e4,  e5,  e6, e7,
                                 e8, e9, e10, e11, e12, e13, e14};
  for (auto const& e : exprs) {
    hash_set.insert(get_std_hash(e));
  }
  EXPECT_EQ(hash_set.size(), exprs.size());
}

// Confirm that numeric_limits is appropriately specialized for Expression.
// We'll just spot-test a few values, since our implementation is trivially
// forwarding to numeric_limits<double>.
TEST_F(SymbolicExpressionTest, NumericLimits) {
  using std::numeric_limits;
  using Limits = numeric_limits<Expression>;

  const Expression num_eps = Limits::epsilon();
  ASSERT_TRUE(is_constant(num_eps));
  EXPECT_EQ(get_constant_value(num_eps), numeric_limits<double>::epsilon());

  const Expression num_min = Limits::min();
  ASSERT_TRUE(is_constant(num_min));
  EXPECT_EQ(get_constant_value(num_min), numeric_limits<double>::min());

  const Expression num_infinity = Limits::infinity();
  EXPECT_EQ(num_infinity.to_string(), "inf");
}

TEST_F(SymbolicExpressionTest, UnaryPlus) {
  EXPECT_PRED2(ExprEqual, c3_, +c3_);
  EXPECT_PRED2(ExprEqual, Expression(var_x_), +var_x_);
}

// TODO(jwnimmer-tri) These tests should probably live in symbolic_formula_test.
//
// Confirm that Eigen::numext::{not_,}equal_strict are appropriately
// specialized for Expression.
// We only need a limited set of cases because if the specialization doesn't
// exist, this would result in a compile error.
// This function was only introduced in eigen 3.3.5. Therefore, we only want to
// test if the eigen version is at least that.
#if EIGEN_VERSION_AT_LEAST(3, 3, 5)
TEST_F(SymbolicExpressionTest, EigenEqualStrict) {
  EXPECT_TRUE(Eigen::numext::equal_strict(c3_, c3_));
  EXPECT_FALSE(Eigen::numext::equal_strict(c3_, c4_));

  // Check our special-case zero handling.
  EXPECT_TRUE(Eigen::numext::equal_strict(zero_, zero_));
  EXPECT_FALSE(Eigen::numext::equal_strict(zero_, one_));
  EXPECT_FALSE(Eigen::numext::equal_strict(one_, zero_));
  EXPECT_FALSE(Eigen::numext::equal_strict(zero_, x_));
  EXPECT_FALSE(Eigen::numext::equal_strict(x_, zero_));
  EXPECT_THROW(Eigen::numext::equal_strict(x_, y_), std::exception);
}

TEST_F(SymbolicExpressionTest, EigenNotEqualStrict) {
  EXPECT_TRUE(Eigen::numext::not_equal_strict(c3_, c4_));
  EXPECT_FALSE(Eigen::numext::not_equal_strict(c3_, c3_));

  // Check our special-case zero handling.
  EXPECT_FALSE(Eigen::numext::not_equal_strict(zero_, zero_));
  EXPECT_TRUE(Eigen::numext::not_equal_strict(zero_, one_));
  EXPECT_TRUE(Eigen::numext::not_equal_strict(one_, zero_));
  EXPECT_TRUE(Eigen::numext::not_equal_strict(zero_, x_));
  EXPECT_TRUE(Eigen::numext::not_equal_strict(x_, zero_));
  EXPECT_THROW(Eigen::numext::not_equal_strict(x_, y_), std::exception);
}
#endif

// Confirm the other Eigen::numext specializations:
//  - isfinite
//  - isnan
//  - isinf
// They all trivially forward to our own functions.
TEST_F(SymbolicExpressionTest, EigenNumext) {
  // isnan is only valid for non-NaN Expressions. Trying to evaluate
  // a NaN expression will throw an exception. So we can't check that.
  EXPECT_FALSE(Eigen::numext::isnan(one_));

  const Expression num_infinity = std::numeric_limits<Expression>::infinity();

  EXPECT_FALSE(Eigen::numext::isinf(one_));
  EXPECT_TRUE(Eigen::numext::isinf(num_infinity));

  EXPECT_TRUE(Eigen::numext::isfinite(one_));
  EXPECT_FALSE(Eigen::numext::isfinite(num_infinity));
}

TEST_F(SymbolicExpressionTest, UnaryMinus) {
  EXPECT_PRED2(ExprEqual, -Expression(var_x_), -var_x_);
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
  EXPECT_TRUE(std::is_nothrow_move_constructible_v<Expression>);
}

TEST_F(SymbolicExpressionTest, Log) {
  EXPECT_DOUBLE_EQ(log(pi_).Evaluate(), std::log(M_PI));
  EXPECT_DOUBLE_EQ(log(one_).Evaluate(), std::log(1.0));
  EXPECT_DOUBLE_EQ(log(zero_).Evaluate(), std::log(0.0));
  EXPECT_THROW(log(neg_one_).Evaluate(), domain_error);
  EXPECT_THROW(log(neg_pi_).Evaluate(), domain_error);
  const Expression e{log(x_ * y_ * pi_) + log(x_) + log(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::log(2 * 3.2 * M_PI) + std::log(2) + std::log(3.2));
  EXPECT_EQ((log(x_)).to_string(), "log(x)");
}

TEST_F(SymbolicExpressionTest, Abs) {
  EXPECT_DOUBLE_EQ(abs(pi_).Evaluate(), std::fabs(M_PI));
  EXPECT_DOUBLE_EQ(abs(one_).Evaluate(), std::fabs(1.0));
  EXPECT_DOUBLE_EQ(abs(zero_).Evaluate(), std::fabs(0.0));
  EXPECT_DOUBLE_EQ(abs(neg_one_).Evaluate(), std::fabs(-1.0));
  EXPECT_DOUBLE_EQ(abs(neg_pi_).Evaluate(), std::fabs(-M_PI));
  const Expression e{abs(x_ * y_ * pi_) + abs(x_) + abs(y_)};
  const Environment env{{var_x_, -2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::fabs(-2 * 3.2 * M_PI) +
                                        std::fabs(-2.0) + std::fabs(3.2));
  EXPECT_EQ((abs(x_)).to_string(), "abs(x)");
}

TEST_F(SymbolicExpressionTest, Exp) {
  EXPECT_DOUBLE_EQ(exp(pi_).Evaluate(), std::exp(M_PI));
  EXPECT_DOUBLE_EQ(exp(one_).Evaluate(), std::exp(1));
  EXPECT_DOUBLE_EQ(exp(zero_).Evaluate(), std::exp(0));
  EXPECT_DOUBLE_EQ(exp(neg_one_).Evaluate(), std::exp(-1));
  EXPECT_DOUBLE_EQ(exp(neg_pi_).Evaluate(), std::exp(-M_PI));

  const Expression e{exp(x_ * y_ * pi_) + exp(x_) + exp(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::exp(2 * 3.2 * M_PI) + std::exp(2.0) + std::exp(3.2));
  EXPECT_EQ((exp(x_)).to_string(), "exp(x)");
}

TEST_F(SymbolicExpressionTest, Sqrt1) {
  // sqrt(x * x) => |x|
  EXPECT_PRED2(ExprEqual, sqrt(x_plus_y_ * x_plus_y_), abs(x_plus_y_));
}

TEST_F(SymbolicExpressionTest, Sqrt2) {
  EXPECT_DOUBLE_EQ(sqrt(pi_).Evaluate(), std::sqrt(M_PI));
  EXPECT_DOUBLE_EQ(sqrt(one_).Evaluate(), std::sqrt(1.0));
  EXPECT_DOUBLE_EQ(sqrt(zero_).Evaluate(), std::sqrt(0.0));
  EXPECT_THROW(sqrt(neg_one_).Evaluate(), domain_error);
  EXPECT_THROW(sqrt(neg_pi_).Evaluate(), domain_error);

  const Expression e{sqrt(x_ * y_ * pi_) + sqrt(x_) + sqrt(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::sqrt(2 * 3.2 * M_PI) + std::sqrt(2.0) + std::sqrt(3.2));
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
  EXPECT_DOUBLE_EQ(pow(pi_, pi_).Evaluate(), std::pow(M_PI, M_PI));
  EXPECT_DOUBLE_EQ(pow(pi_, one_).Evaluate(), std::pow(M_PI, 1));
  EXPECT_DOUBLE_EQ(pow(pi_, two_).Evaluate(), std::pow(M_PI, 2));
  EXPECT_DOUBLE_EQ(pow(pi_, zero_).Evaluate(), std::pow(M_PI, 0));
  EXPECT_DOUBLE_EQ(pow(pi_, neg_one_).Evaluate(), std::pow(M_PI, -1));
  EXPECT_DOUBLE_EQ(pow(pi_, neg_pi_).Evaluate(), std::pow(M_PI, -M_PI));

  EXPECT_DOUBLE_EQ(pow(one_, pi_).Evaluate(), std::pow(1, M_PI));
  EXPECT_DOUBLE_EQ(pow(one_, one_).Evaluate(), std::pow(1, 1));
  EXPECT_DOUBLE_EQ(pow(one_, two_).Evaluate(), std::pow(1, 2));
  EXPECT_DOUBLE_EQ(pow(one_, zero_).Evaluate(), std::pow(1, 0));
  EXPECT_DOUBLE_EQ(pow(one_, neg_one_).Evaluate(), std::pow(1, -1));
  EXPECT_DOUBLE_EQ(pow(one_, neg_pi_).Evaluate(), std::pow(1, -M_PI));

  EXPECT_DOUBLE_EQ(pow(two_, pi_).Evaluate(), std::pow(2, M_PI));
  EXPECT_DOUBLE_EQ(pow(two_, one_).Evaluate(), std::pow(2, 1));
  EXPECT_DOUBLE_EQ(pow(two_, two_).Evaluate(), std::pow(2, 2));
  EXPECT_DOUBLE_EQ(pow(two_, zero_).Evaluate(), std::pow(2, 0));
  EXPECT_DOUBLE_EQ(pow(two_, neg_one_).Evaluate(), std::pow(2, -1));
  EXPECT_DOUBLE_EQ(pow(two_, neg_pi_).Evaluate(), std::pow(2, -M_PI));

  EXPECT_DOUBLE_EQ(pow(zero_, pi_).Evaluate(), std::pow(0, M_PI));
  EXPECT_DOUBLE_EQ(pow(zero_, one_).Evaluate(), std::pow(0, 1));
  EXPECT_DOUBLE_EQ(pow(zero_, two_).Evaluate(), std::pow(0, 2));
  EXPECT_DOUBLE_EQ(pow(zero_, zero_).Evaluate(), std::pow(0, 0));
  EXPECT_DOUBLE_EQ(pow(zero_, neg_one_).Evaluate(), std::pow(0, -1));
  EXPECT_DOUBLE_EQ(pow(zero_, neg_pi_).Evaluate(), std::pow(0, -M_PI));

  EXPECT_THROW(pow(neg_one_, pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_one_, one_).Evaluate(), std::pow(-1, 1));
  EXPECT_DOUBLE_EQ(pow(neg_one_, two_).Evaluate(), std::pow(-1, 2));
  EXPECT_DOUBLE_EQ(pow(neg_one_, zero_).Evaluate(), std::pow(-1, 0));
  EXPECT_DOUBLE_EQ(pow(neg_one_, neg_one_).Evaluate(), std::pow(-1, -1));
  EXPECT_THROW(pow(neg_one_, neg_pi_).Evaluate(), domain_error);

  EXPECT_THROW(pow(neg_pi_, pi_).Evaluate(), domain_error);
  EXPECT_DOUBLE_EQ(pow(neg_pi_, one_).Evaluate(), std::pow(-M_PI, 1));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, two_).Evaluate(), std::pow(-M_PI, 2));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, zero_).Evaluate(), std::pow(-M_PI, 0));
  EXPECT_DOUBLE_EQ(pow(neg_pi_, neg_one_).Evaluate(), std::pow(-M_PI, -1));
  EXPECT_THROW(pow(neg_pi_, neg_pi_).Evaluate(), domain_error);

  const Expression e1{pow(x_ * y_ * pi_, x_ + y_ + pi_)};
  const Expression e2{(pow(x_, 2) * pow(y_, 2) * pow(x_, y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e1.Evaluate(env), std::pow(2 * 3.2 * M_PI, 2 + 3.2 + M_PI));
  EXPECT_DOUBLE_EQ(e2.Evaluate(env),
                   std::pow(2, 2) * std::pow(3.2, 2) * std::pow(2, 3.2));
}

TEST_F(SymbolicExpressionTest, Sin) {
  EXPECT_DOUBLE_EQ(sin(pi_).Evaluate(), std::sin(M_PI));
  EXPECT_DOUBLE_EQ(sin(one_).Evaluate(), std::sin(1));
  EXPECT_DOUBLE_EQ(sin(two_).Evaluate(), std::sin(2));
  EXPECT_DOUBLE_EQ(sin(zero_).Evaluate(), std::sin(0));
  EXPECT_DOUBLE_EQ(sin(neg_one_).Evaluate(), std::sin(-1));
  EXPECT_DOUBLE_EQ(sin(neg_pi_).Evaluate(), std::sin(-M_PI));

  const Expression e{sin(x_ * y_ * pi_) + sin(x_) + sin(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::sin(2 * 3.2 * M_PI) + std::sin(2) + std::sin(3.2));
  EXPECT_EQ((sin(x_)).to_string(), "sin(x)");
}

TEST_F(SymbolicExpressionTest, Cos) {
  EXPECT_DOUBLE_EQ(cos(pi_).Evaluate(), std::cos(M_PI));
  EXPECT_DOUBLE_EQ(cos(one_).Evaluate(), std::cos(1));
  EXPECT_DOUBLE_EQ(cos(two_).Evaluate(), std::cos(2));
  EXPECT_DOUBLE_EQ(cos(zero_).Evaluate(), std::cos(0));
  EXPECT_DOUBLE_EQ(cos(neg_one_).Evaluate(), std::cos(-1));
  EXPECT_DOUBLE_EQ(cos(neg_pi_).Evaluate(), std::cos(-M_PI));
  const Expression e{cos(x_ * y_ * pi_) + cos(x_) + cos(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::cos(2 * 3.2 * M_PI) + std::cos(2) + std::cos(3.2));
  EXPECT_EQ((cos(x_)).to_string(), "cos(x)");
}

TEST_F(SymbolicExpressionTest, Tan) {
  EXPECT_DOUBLE_EQ(tan(pi_).Evaluate(), std::tan(M_PI));
  EXPECT_DOUBLE_EQ(tan(one_).Evaluate(), std::tan(1));
  EXPECT_DOUBLE_EQ(tan(two_).Evaluate(), std::tan(2));
  EXPECT_DOUBLE_EQ(tan(zero_).Evaluate(), std::tan(0));
  EXPECT_DOUBLE_EQ(tan(neg_one_).Evaluate(), std::tan(-1));
  EXPECT_DOUBLE_EQ(tan(neg_pi_).Evaluate(), std::tan(-M_PI));

  const Expression e{tan(x_ * y_ * pi_) + tan(x_) + tan(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::tan(2 * 3.2 * M_PI) + std::tan(2) + std::tan(3.2));
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
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::asin(0.2 * 0.3 * M_PI) +
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
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::acos(0.2 * 0.3 * M_PI) +
                                        std::acos(0.2) + std::acos(0.3));
  EXPECT_EQ((acos(x_)).to_string(), "acos(x)");
}

TEST_F(SymbolicExpressionTest, Atan) {
  EXPECT_DOUBLE_EQ(atan(pi_).Evaluate(), std::atan(M_PI));
  EXPECT_DOUBLE_EQ(atan(one_).Evaluate(), std::atan(1));
  EXPECT_DOUBLE_EQ(atan(two_).Evaluate(), std::atan(2));
  EXPECT_DOUBLE_EQ(atan(zero_).Evaluate(), std::atan(0));
  EXPECT_DOUBLE_EQ(atan(neg_one_).Evaluate(), std::atan(-1));
  EXPECT_DOUBLE_EQ(atan(neg_pi_).Evaluate(), std::atan(-M_PI));

  const Expression e{atan(x_ * y_ * pi_) + atan(x_) + atan(y_)};
  const Environment env{{var_x_, 0.2}, {var_y_, 0.3}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::atan(0.2 * 0.3 * M_PI) +
                                        std::atan(0.2) + std::atan(0.3));
  EXPECT_EQ((atan(x_)).to_string(), "atan(x)");
}

TEST_F(SymbolicExpressionTest, Atan2) {
  EXPECT_DOUBLE_EQ(atan2(pi_, pi_).Evaluate(), std::atan2(M_PI, M_PI));
  EXPECT_DOUBLE_EQ(atan2(pi_, one_).Evaluate(), std::atan2(M_PI, 1));
  EXPECT_DOUBLE_EQ(atan2(pi_, two_).Evaluate(), std::atan2(M_PI, 2));
  EXPECT_DOUBLE_EQ(atan2(pi_, zero_).Evaluate(), std::atan2(M_PI, 0));
  EXPECT_DOUBLE_EQ(atan2(pi_, neg_one_).Evaluate(), std::atan2(M_PI, -1));
  EXPECT_DOUBLE_EQ(atan2(pi_, neg_pi_).Evaluate(), std::atan2(M_PI, -M_PI));

  EXPECT_DOUBLE_EQ(atan2(one_, pi_).Evaluate(), std::atan2(1, M_PI));
  EXPECT_DOUBLE_EQ(atan2(one_, one_).Evaluate(), std::atan2(1, 1));
  EXPECT_DOUBLE_EQ(atan2(one_, two_).Evaluate(), std::atan2(1, 2));
  EXPECT_DOUBLE_EQ(atan2(one_, zero_).Evaluate(), std::atan2(1, 0));
  EXPECT_DOUBLE_EQ(atan2(one_, neg_one_).Evaluate(), std::atan2(1, -1));
  EXPECT_DOUBLE_EQ(atan2(one_, neg_pi_).Evaluate(), std::atan2(1, -M_PI));

  EXPECT_DOUBLE_EQ(atan2(two_, pi_).Evaluate(), std::atan2(2, M_PI));
  EXPECT_DOUBLE_EQ(atan2(two_, one_).Evaluate(), std::atan2(2, 1));
  EXPECT_DOUBLE_EQ(atan2(two_, two_).Evaluate(), std::atan2(2, 2));
  EXPECT_DOUBLE_EQ(atan2(two_, zero_).Evaluate(), std::atan2(2, 0));
  EXPECT_DOUBLE_EQ(atan2(two_, neg_one_).Evaluate(), std::atan2(2, -1));
  EXPECT_DOUBLE_EQ(atan2(two_, neg_pi_).Evaluate(), std::atan2(2, -M_PI));

  EXPECT_DOUBLE_EQ(atan2(zero_, pi_).Evaluate(), std::atan2(0, M_PI));
  EXPECT_DOUBLE_EQ(atan2(zero_, one_).Evaluate(), std::atan2(0, 1));
  EXPECT_DOUBLE_EQ(atan2(zero_, two_).Evaluate(), std::atan2(0, 2));
  EXPECT_DOUBLE_EQ(atan2(zero_, zero_).Evaluate(), std::atan2(0, 0));
  EXPECT_DOUBLE_EQ(atan2(zero_, neg_one_).Evaluate(), std::atan2(0, -1));
  EXPECT_DOUBLE_EQ(atan2(zero_, neg_pi_).Evaluate(), std::atan2(0, -M_PI));

  EXPECT_DOUBLE_EQ(atan2(neg_one_, pi_).Evaluate(), std::atan2(-1, M_PI));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, one_).Evaluate(), std::atan2(-1, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, two_).Evaluate(), std::atan2(-1, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, zero_).Evaluate(), std::atan2(-1, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, neg_one_).Evaluate(), std::atan2(-1, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_one_, neg_pi_).Evaluate(), std::atan2(-1, -M_PI));

  EXPECT_DOUBLE_EQ(atan2(neg_pi_, pi_).Evaluate(), std::atan2(-M_PI, M_PI));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, one_).Evaluate(), std::atan2(-M_PI, 1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, two_).Evaluate(), std::atan2(-M_PI, 2));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, zero_).Evaluate(), std::atan2(-M_PI, 0));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, neg_one_).Evaluate(), std::atan2(-M_PI, -1));
  EXPECT_DOUBLE_EQ(atan2(neg_pi_, neg_pi_).Evaluate(),
                   std::atan2(-M_PI, -M_PI));

  const Expression e{atan2(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::atan2(2 * 3.2 * M_PI, std::sin(2) + std::sin(3.2)));
  EXPECT_EQ((atan2(x_, y_)).to_string(), "atan2(x, y)");
}

TEST_F(SymbolicExpressionTest, Sinh) {
  EXPECT_DOUBLE_EQ(sinh(pi_).Evaluate(), std::sinh(M_PI));
  EXPECT_DOUBLE_EQ(sinh(one_).Evaluate(), std::sinh(1));
  EXPECT_DOUBLE_EQ(sinh(two_).Evaluate(), std::sinh(2));
  EXPECT_DOUBLE_EQ(sinh(zero_).Evaluate(), std::sinh(0));
  EXPECT_DOUBLE_EQ(sinh(neg_one_).Evaluate(), std::sinh(-1));
  EXPECT_DOUBLE_EQ(sinh(neg_pi_).Evaluate(), std::sinh(-M_PI));

  const Expression e{sinh(x_ * y_ * pi_) + sinh(x_) + sinh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::sinh(2 * 3.2 * M_PI) + std::sinh(2) + std::sinh(3.2));
  EXPECT_EQ((sinh(x_)).to_string(), "sinh(x)");
}

TEST_F(SymbolicExpressionTest, Cosh) {
  EXPECT_DOUBLE_EQ(cosh(pi_).Evaluate(), std::cosh(M_PI));
  EXPECT_DOUBLE_EQ(cosh(one_).Evaluate(), std::cosh(1));
  EXPECT_DOUBLE_EQ(cosh(two_).Evaluate(), std::cosh(2));
  EXPECT_DOUBLE_EQ(cosh(zero_).Evaluate(), std::cosh(0));
  EXPECT_DOUBLE_EQ(cosh(neg_one_).Evaluate(), std::cosh(-1));
  EXPECT_DOUBLE_EQ(cosh(neg_pi_).Evaluate(), std::cosh(-M_PI));

  const Expression e{cosh(x_ * y_ * pi_) + cosh(x_) + cosh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::cosh(2 * 3.2 * M_PI) + std::cosh(2) + std::cosh(3.2));
  EXPECT_EQ((cosh(x_)).to_string(), "cosh(x)");
}

TEST_F(SymbolicExpressionTest, Tanh) {
  EXPECT_DOUBLE_EQ(tanh(pi_).Evaluate(), std::tanh(M_PI));
  EXPECT_DOUBLE_EQ(tanh(one_).Evaluate(), std::tanh(1));
  EXPECT_DOUBLE_EQ(tanh(two_).Evaluate(), std::tanh(2));
  EXPECT_DOUBLE_EQ(tanh(zero_).Evaluate(), std::tanh(0));
  EXPECT_DOUBLE_EQ(tanh(neg_one_).Evaluate(), std::tanh(-1));
  EXPECT_DOUBLE_EQ(tanh(neg_pi_).Evaluate(), std::tanh(-M_PI));

  const Expression e{tanh(x_ * y_ * pi_) + tanh(x_) + tanh(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::tanh(2 * 3.2 * M_PI) + std::tanh(2) + std::tanh(3.2));
  EXPECT_EQ((tanh(x_)).to_string(), "tanh(x)");
}

TEST_F(SymbolicExpressionTest, Min1) {
  // min(E, E) -> E
  EXPECT_PRED2(ExprEqual, min(x_plus_y_, x_plus_y_), x_plus_y_);
}

TEST_F(SymbolicExpressionTest, Min2) {
  EXPECT_DOUBLE_EQ(min(pi_, pi_).Evaluate(), std::min(M_PI, M_PI));
  EXPECT_DOUBLE_EQ(min(pi_, one_).Evaluate(), std::min(M_PI, 1.0));
  EXPECT_DOUBLE_EQ(min(pi_, two_).Evaluate(), std::min(M_PI, 2.0));
  EXPECT_DOUBLE_EQ(min(pi_, zero_).Evaluate(), std::min(M_PI, 0.0));
  EXPECT_DOUBLE_EQ(min(pi_, neg_one_).Evaluate(), std::min(M_PI, -1.0));
  EXPECT_DOUBLE_EQ(min(pi_, neg_pi_).Evaluate(), std::min(M_PI, -M_PI));

  EXPECT_DOUBLE_EQ(min(one_, pi_).Evaluate(), std::min(1.0, M_PI));
  EXPECT_DOUBLE_EQ(min(one_, one_).Evaluate(), std::min(1.0, 1.0));
  EXPECT_DOUBLE_EQ(min(one_, two_).Evaluate(), std::min(1.0, 2.0));
  EXPECT_DOUBLE_EQ(min(one_, zero_).Evaluate(), std::min(1.0, 0.0));
  EXPECT_DOUBLE_EQ(min(one_, neg_one_).Evaluate(), std::min(1.0, -1.0));
  EXPECT_DOUBLE_EQ(min(one_, neg_pi_).Evaluate(), std::min(1.0, -M_PI));

  EXPECT_DOUBLE_EQ(min(two_, pi_).Evaluate(), std::min(2.0, M_PI));
  EXPECT_DOUBLE_EQ(min(two_, one_).Evaluate(), std::min(2.0, 1.0));
  EXPECT_DOUBLE_EQ(min(two_, two_).Evaluate(), std::min(2.0, 2.0));
  EXPECT_DOUBLE_EQ(min(two_, zero_).Evaluate(), std::min(2.0, 0.0));
  EXPECT_DOUBLE_EQ(min(two_, neg_one_).Evaluate(), std::min(2.0, -1.0));
  EXPECT_DOUBLE_EQ(min(two_, neg_pi_).Evaluate(), std::min(2.0, -M_PI));

  EXPECT_DOUBLE_EQ(min(zero_, pi_).Evaluate(), std::min(0.0, M_PI));
  EXPECT_DOUBLE_EQ(min(zero_, one_).Evaluate(), std::min(0.0, 1.0));
  EXPECT_DOUBLE_EQ(min(zero_, two_).Evaluate(), std::min(0.0, 2.0));
  EXPECT_DOUBLE_EQ(min(zero_, zero_).Evaluate(), std::min(0.0, 0.0));
  EXPECT_DOUBLE_EQ(min(zero_, neg_one_).Evaluate(), std::min(0.0, -1.0));
  EXPECT_DOUBLE_EQ(min(zero_, neg_pi_).Evaluate(), std::min(0.0, -M_PI));

  EXPECT_DOUBLE_EQ(min(neg_one_, pi_).Evaluate(), std::min(-1.0, M_PI));
  EXPECT_DOUBLE_EQ(min(neg_one_, one_).Evaluate(), std::min(-1.0, 1.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, two_).Evaluate(), std::min(-1.0, 2.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, zero_).Evaluate(), std::min(-1.0, 0.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, neg_one_).Evaluate(), std::min(-1.0, -1.0));
  EXPECT_DOUBLE_EQ(min(neg_one_, neg_pi_).Evaluate(), std::min(-1.0, -M_PI));

  EXPECT_DOUBLE_EQ(min(neg_pi_, pi_).Evaluate(), std::min(-M_PI, M_PI));
  EXPECT_DOUBLE_EQ(min(neg_pi_, one_).Evaluate(), std::min(-M_PI, 1.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, two_).Evaluate(), std::min(-M_PI, 2.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, zero_).Evaluate(), std::min(-M_PI, 0.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, neg_one_).Evaluate(), std::min(-M_PI, -1.0));
  EXPECT_DOUBLE_EQ(min(neg_pi_, neg_pi_).Evaluate(), std::min(-M_PI, -M_PI));

  const Expression e{min(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::min(2 * 3.2 * M_PI, std::sin(2) + std::sin(3.2)));
  EXPECT_EQ((min(x_, y_)).to_string(), "min(x, y)");
}

TEST_F(SymbolicExpressionTest, Max1) {
  // max(E, E) -> E
  EXPECT_PRED2(ExprEqual, max(x_plus_y_, x_plus_y_), x_plus_y_);
}

TEST_F(SymbolicExpressionTest, Max2) {
  EXPECT_DOUBLE_EQ(max(pi_, pi_).Evaluate(), std::max(M_PI, M_PI));
  EXPECT_DOUBLE_EQ(max(pi_, one_).Evaluate(), std::max(M_PI, 1.0));
  EXPECT_DOUBLE_EQ(max(pi_, two_).Evaluate(), std::max(M_PI, 2.0));
  EXPECT_DOUBLE_EQ(max(pi_, zero_).Evaluate(), std::max(M_PI, 0.0));
  EXPECT_DOUBLE_EQ(max(pi_, neg_one_).Evaluate(), std::max(M_PI, -1.0));
  EXPECT_DOUBLE_EQ(max(pi_, neg_pi_).Evaluate(), std::max(M_PI, -M_PI));

  EXPECT_DOUBLE_EQ(max(one_, pi_).Evaluate(), std::max(1.0, M_PI));
  EXPECT_DOUBLE_EQ(max(one_, one_).Evaluate(), std::max(1.0, 1.0));
  EXPECT_DOUBLE_EQ(max(one_, two_).Evaluate(), std::max(1.0, 2.0));
  EXPECT_DOUBLE_EQ(max(one_, zero_).Evaluate(), std::max(1.0, 0.0));
  EXPECT_DOUBLE_EQ(max(one_, neg_one_).Evaluate(), std::max(1.0, -1.0));
  EXPECT_DOUBLE_EQ(max(one_, neg_pi_).Evaluate(), std::max(1.0, -M_PI));

  EXPECT_DOUBLE_EQ(max(two_, pi_).Evaluate(), std::max(2.0, M_PI));
  EXPECT_DOUBLE_EQ(max(two_, one_).Evaluate(), std::max(2.0, 1.0));
  EXPECT_DOUBLE_EQ(max(two_, two_).Evaluate(), std::max(2.0, 2.0));
  EXPECT_DOUBLE_EQ(max(two_, zero_).Evaluate(), std::max(2.0, 0.0));
  EXPECT_DOUBLE_EQ(max(two_, neg_one_).Evaluate(), std::max(2.0, -1.0));
  EXPECT_DOUBLE_EQ(max(two_, neg_pi_).Evaluate(), std::max(2.0, -M_PI));

  EXPECT_DOUBLE_EQ(max(zero_, pi_).Evaluate(), std::max(0.0, M_PI));
  EXPECT_DOUBLE_EQ(max(zero_, one_).Evaluate(), std::max(0.0, 1.0));
  EXPECT_DOUBLE_EQ(max(zero_, two_).Evaluate(), std::max(0.0, 2.0));
  EXPECT_DOUBLE_EQ(max(zero_, zero_).Evaluate(), std::max(0.0, 0.0));
  EXPECT_DOUBLE_EQ(max(zero_, neg_one_).Evaluate(), std::max(0.0, -1.0));
  EXPECT_DOUBLE_EQ(max(zero_, neg_pi_).Evaluate(), std::max(0.0, -M_PI));

  EXPECT_DOUBLE_EQ(max(neg_one_, pi_).Evaluate(), std::max(-1.0, M_PI));
  EXPECT_DOUBLE_EQ(max(neg_one_, one_).Evaluate(), std::max(-1.0, 1.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, two_).Evaluate(), std::max(-1.0, 2.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, zero_).Evaluate(), std::max(-1.0, 0.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, neg_one_).Evaluate(), std::max(-1.0, -1.0));
  EXPECT_DOUBLE_EQ(max(neg_one_, neg_pi_).Evaluate(), std::max(-1.0, -M_PI));

  EXPECT_DOUBLE_EQ(max(neg_pi_, pi_).Evaluate(), std::max(-M_PI, M_PI));
  EXPECT_DOUBLE_EQ(max(neg_pi_, one_).Evaluate(), std::max(-M_PI, 1.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, two_).Evaluate(), std::max(-M_PI, 2.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, zero_).Evaluate(), std::max(-M_PI, 0.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, neg_one_).Evaluate(), std::max(-M_PI, -1.0));
  EXPECT_DOUBLE_EQ(max(neg_pi_, neg_pi_).Evaluate(), std::max(-M_PI, -M_PI));

  const Expression e{max(x_ * y_ * pi_, sin(x_) + sin(y_))};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::max(2 * 3.2 * M_PI, std::sin(2) + std::sin(3.2)));
  EXPECT_EQ((max(x_, y_)).to_string(), "max(x, y)");
}

TEST_F(SymbolicExpressionTest, Ceil) {
  EXPECT_DOUBLE_EQ(ceil(pi_).Evaluate(), std::ceil(M_PI));
  EXPECT_DOUBLE_EQ(ceil(one_).Evaluate(), std::ceil(1));
  EXPECT_DOUBLE_EQ(ceil(two_).Evaluate(), std::ceil(2));
  EXPECT_DOUBLE_EQ(ceil(zero_).Evaluate(), std::ceil(0));
  EXPECT_DOUBLE_EQ(ceil(neg_one_).Evaluate(), std::ceil(-1));
  EXPECT_DOUBLE_EQ(ceil(neg_pi_).Evaluate(), std::ceil(-M_PI));

  const Expression e{ceil(x_ * y_ * pi_) + ceil(x_) + ceil(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env),
                   std::ceil(2 * 3.2 * M_PI) + std::ceil(2) + std::ceil(3.2));
  EXPECT_EQ((ceil(x_)).to_string(), "ceil(x)");
}

TEST_F(SymbolicExpressionTest, Floor) {
  EXPECT_DOUBLE_EQ(floor(pi_).Evaluate(), std::floor(M_PI));
  EXPECT_DOUBLE_EQ(floor(one_).Evaluate(), std::floor(1));
  EXPECT_DOUBLE_EQ(floor(two_).Evaluate(), std::floor(2));
  EXPECT_DOUBLE_EQ(floor(zero_).Evaluate(), std::floor(0));
  EXPECT_DOUBLE_EQ(floor(neg_one_).Evaluate(), std::floor(-1));
  EXPECT_DOUBLE_EQ(floor(neg_pi_).Evaluate(), std::floor(-M_PI));

  const Expression e{floor(x_ * y_ * pi_) + floor(x_) + floor(y_)};
  const Environment env{{var_x_, 2}, {var_y_, 3.2}};
  EXPECT_DOUBLE_EQ(e.Evaluate(env), std::floor(2 * 3.2 * M_PI) + std::floor(2) +
                                        std::floor(3.2));
  EXPECT_EQ((floor(x_)).to_string(), "floor(x)");
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

TEST_F(SymbolicExpressionTest,
       UninterpretedFunction_GetVariables_GetName_GetArguments) {
  const Expression uf1{uninterpreted_function("uf1", {})};
  EXPECT_TRUE(uf1.GetVariables().empty());
  EXPECT_EQ(get_uninterpreted_function_name(uf1), "uf1");
  EXPECT_TRUE(get_uninterpreted_function_arguments(uf1).empty());

  const Expression uf2{uninterpreted_function("uf2", {var_x_, var_y_})};
  EXPECT_EQ(get_uninterpreted_function_name(uf2), "uf2");
  const Variables vars_in_uf2{uf2.GetVariables()};
  EXPECT_EQ(vars_in_uf2.size(), 2);
  EXPECT_TRUE(vars_in_uf2.include(var_x_));
  EXPECT_TRUE(vars_in_uf2.include(var_y_));

  const vector<Expression> arguments{sin(x_), cos(y_)};
  const Expression uf3{uninterpreted_function("uf3", arguments)};
  const vector<Expression>& the_arguments{
      get_uninterpreted_function_arguments(uf3)};
  EXPECT_EQ(arguments.size(), the_arguments.size());
  EXPECT_PRED2(ExprEqual, arguments[0], the_arguments[0]);
  EXPECT_PRED2(ExprEqual, arguments[1], the_arguments[1]);
}

TEST_F(SymbolicExpressionTest, UninterpretedFunctionEvaluate) {
  const Expression uf1{uninterpreted_function("uf1", {})};
  const Expression uf2{uninterpreted_function("uf2", {var_x_, var_y_})};
  EXPECT_THROW(uf1.Evaluate(), std::runtime_error);
  EXPECT_THROW(uf2.Evaluate(), std::runtime_error);
}

TEST_F(SymbolicExpressionTest, UninterpretedFunctionEqual) {
  const Expression uf1{uninterpreted_function("name1", {x_, y_ + z_})};
  const Expression uf2{uninterpreted_function("name1", {x_, y_ + z_})};
  EXPECT_TRUE(uf1.EqualTo(uf2));

  const Expression uf3{uninterpreted_function("name2", {x_, y_ + z_})};
  EXPECT_FALSE(uf1.EqualTo(uf3));
  const Expression uf4{uninterpreted_function("name1", {y_, y_ + z_})};
  EXPECT_FALSE(uf1.EqualTo(uf4));
  const Expression uf5{uninterpreted_function("name1", {x_, z_})};
  EXPECT_FALSE(uf1.EqualTo(uf5));
  const Expression uf6{uninterpreted_function("name1", {x_, y_ + z_, 3.0})};
  EXPECT_FALSE(uf1.EqualTo(uf6));
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
  const Expression e3{M_PI * x_ * pow(y_, M_E)};
  const Expression e4{M_E + x_ + M_PI * y_};

  EXPECT_EQ(e1.to_string(), "sin((x + (y * z)))");
  EXPECT_EQ(e2.to_string(), "cos(((pow(y, 2) * z) + pow(x, 2)))");
  EXPECT_EQ(e3.to_string(),
            "(3.1415926535897931 * x * pow(y, 2.7182818284590451))");
  EXPECT_EQ(e4.to_string(),
            "(2.7182818284590451 + x + 3.1415926535897931 * y)");
  EXPECT_EQ(e_uf_.to_string(), "uf(x, y)");
}

TEST_F(SymbolicExpressionTest, EvaluatePartial) {
  // e = xy - 5yz + 10xz.
  const Expression e{x_ * y_ - 5 * y_ * z_ + 10 * x_ * z_};

  // e1 = e[x ↦ 3] = 3y - 5yz + 30z
  const Expression e1{e.EvaluatePartial({{var_x_, 3}})};
  EXPECT_PRED2(ExprEqual, e1, 3 * y_ - 5 * y_ * z_ + 30 * z_);

  // e2 = e1[y ↦ 5] = 15 - 25z + 30z = 15 + 5z
  const Expression e2{e1.EvaluatePartial({{var_y_, 5}})};
  EXPECT_PRED2(ExprEqual, e2, 15 + 5 * z_);

  // e3 = e1[z ↦ -2] = 15 + (-10) = 5
  const Expression e3{e2.EvaluatePartial({{var_z_, -2}})};
  EXPECT_PRED2(ExprEqual, e3, 5);
}

// Checks for compatibility with a memcpy primitive move operation.
// See https://github.com/RobotLocomotion/drake/issues/5974.
TEST_F(SymbolicExpressionTest, MemcpyKeepsExpressionIntact) {
  for (const Expression& expr : collection_) {
    EXPECT_TRUE(IsMemcpyMovable(expr));
  }
}

TEST_F(SymbolicExpressionTest, ExtractDoubleTest) {
  const Expression e1{10.0};
  EXPECT_EQ(ExtractDoubleOrThrow(e1), 10.0);

  // 'x_' can't be converted to a double value.
  const Expression e2{x_};
  EXPECT_THROW(ExtractDoubleOrThrow(e2), std::exception);

  // 2x - 7 -2x + 2 => -5
  const Expression e3{2 * x_ - 7 - 2 * x_ + 2};
  EXPECT_EQ(ExtractDoubleOrThrow(e3), -5);

  // Literal NaN should come through without an exception during Extract.
  EXPECT_TRUE(std::isnan(ExtractDoubleOrThrow(e_nan_)));

  // Computed NaN should still throw.
  const Expression bogus = zero_ / e_nan_;
  EXPECT_THROW(ExtractDoubleOrThrow(bogus), std::exception);

  // Eigen variant.
  const Vector2<Expression> v1{12.0, 13.0};
  EXPECT_TRUE(
      CompareMatrices(ExtractDoubleOrThrow(v1), Eigen::Vector2d{12.0, 13.0}));

  // Computed NaN should still throw through the Eigen variant.
  const Vector2<Expression> v2{12.0, bogus};
  EXPECT_THROW(ExtractDoubleOrThrow(v2), std::exception);
}

TEST_F(SymbolicExpressionTest, Jacobian) {
  // J1 = (x * y + sin(x)).Jacobian([x, y])
  //    = [y + cos(y), x]
  const Vector2<Variable> vars{var_x_, var_y_};
  const auto J1 = (x_ * y_ + sin(x_)).Jacobian(vars);
  // This should be matched with the non-member function Jacobian.
  const auto J2 = Jacobian(Vector1<Expression>(x_ * y_ + sin(x_)), vars);
  // Checks the sizes.
  EXPECT_EQ(J1.rows(), 1);
  EXPECT_EQ(J2.rows(), 1);
  EXPECT_EQ(J1.cols(), 2);
  EXPECT_EQ(J2.cols(), 2);
  // Checks the elements.
  EXPECT_EQ(J1(0), y_ + cos(x_));
  EXPECT_EQ(J1(1), x_);
  EXPECT_EQ(J2(0), J1(0));
  EXPECT_EQ(J2(1), J1(1));
}

TEST_F(SymbolicExpressionTest, GetDistinctVariables) {
  EXPECT_EQ(GetDistinctVariables(Vector1<Expression>{x_plus_y_}),
            Variables({var_x_, var_y_}));
  EXPECT_EQ(GetDistinctVariables(Vector1<Expression>{x_plus_z_}),
            Variables({var_x_, var_z_}));
  EXPECT_EQ(GetDistinctVariables(Vector2<Expression>{x_plus_y_, x_plus_z_}),
            Variables({var_x_, var_y_, var_z_}));
  EXPECT_EQ(GetDistinctVariables(RowVector2<Expression>{x_plus_z_, e_cos_}),
            Variables({var_x_, var_z_}));
}

TEST_F(SymbolicExpressionTest, TaylorExpand1) {
  // Test TaylorExpand(exp(-x²-y²), {x:1, y:2}, 2).
  const Expression& x{x_};
  const Expression& y{y_};
  const Expression e{exp(-x * x - y * y)};
  const Environment env{{{var_x_, 1}, {var_y_, 2}}};
  const Expression expanded{TaylorExpand(e, env, 2)};
  // Obtained from Matlab.
  const Expression expected{std::exp(-5) *
                            (1 - 2 * (x - 1) - 4 * (y - 2) + pow(x - 1, 2) +
                             8 * (x - 1) * (y - 2) + 7 * pow(y - 2, 2))};
  // The difference should be close to zero. We sample a few points around (1,
  // 2) and test.
  const vector<Environment> test_envs{{{{var_x_, 0}, {var_y_, 0}}},
                                      {{{var_x_, 2}, {var_y_, 3}}},
                                      {{{var_x_, 0}, {var_y_, 3}}},
                                      {{{var_x_, 2}, {var_y_, 0}}}};
  for (const auto& test_env : test_envs) {
    EXPECT_NEAR((expanded - expected).Evaluate(test_env), 0.0, 1e-10);
  }
}

TEST_F(SymbolicExpressionTest, TaylorExpand2) {
  // Test TaylorExpand(sin(-x² -y²), {x:1, y:2}, 2).
  const Expression& x{x_};
  const Expression& y{y_};
  const Expression e{sin(-x * x - y * y)};
  const Environment env{{{var_x_, 1}, {var_y_, 2}}};
  const Expression expanded{TaylorExpand(e, env, 2)};
  // Obtained from Matlab.
  const Expression expected{8 * sin(5) * (x - 1) * (y - 2) -
                            (cos(5) - 2 * sin(5)) * (x - 1) * (x - 1) -
                            (cos(5) - 8 * sin(5)) * (y - 2) * (y - 2) -
                            2 * cos(5) * (x - 1) - 4 * cos(5) * (y - 2) -
                            sin(5)};
  // The difference should be close to zero. We sample a few points around (1,
  // 2) and test.
  const vector<Environment> test_envs{{{{var_x_, 0}, {var_y_, 0}}},
                                      {{{var_x_, 2}, {var_y_, 3}}},
                                      {{{var_x_, 0}, {var_y_, 3}}},
                                      {{{var_x_, 2}, {var_y_, 0}}}};
  for (const auto& test_env : test_envs) {
    EXPECT_NEAR((expanded - expected).Evaluate(test_env), 0.0, 1e-10);
  }
}

TEST_F(SymbolicExpressionTest, TaylorExpand3) {
  // Test TaylorExpand(sin(-x² - y²) + cos(z), {x:1, y:2, z:3}, 3)
  const Expression& x{x_};
  const Expression& y{y_};
  const Expression& z{z_};
  const Expression e{sin(-pow(x, 2) - pow(y, 2)) + cos(z)};
  const Environment env{{{var_x_, 1}, {var_y_, 2}, {var_z_, 3}}};
  const Expression expanded{TaylorExpand(e, env, 3)};
  // Obtained from Matlab.
  const Expression expected{
      cos(3) - sin(5) + (sin(3) * pow(z - 3, 3)) / 6 -
      (cos(5) - 2 * sin(5)) * pow(x - 1, 2) -
      (cos(5) - 8 * sin(5)) * pow(y - 2, 2) +
      pow(x - 1, 3) * ((4 * cos(5)) / 3 + 2 * sin(5)) +
      pow(y - 2, 3) * ((32 * cos(5)) / 3 + 4 * sin(5)) - 2 * cos(5) * (x - 1) -
      4 * cos(5) * (y - 2) - sin(3) * (z - 3) - (cos(3) * pow(z - 3, 2)) / 2 +
      pow(x - 1, 2) * (y - 2) * (8 * cos(5) + 4 * sin(5)) +
      (x - 1) * pow(y - 2, 2) * (16 * cos(5) + 2 * sin(5)) +
      8 * sin(5) * (x - 1) * (y - 2)};

  // The difference should be close to zero. We sample a few points around (1,
  // 2) and test.
  const vector<Environment> test_envs{
      {{{var_x_, 0}, {var_y_, 0}, {var_z_, 0}}},
      {{{var_x_, 0}, {var_y_, 0}, {var_z_, 3}}},
      {{{var_x_, 0}, {var_y_, 3}, {var_z_, 0}}},
      {{{var_x_, 3}, {var_y_, 0}, {var_z_, 0}}},
      {{{var_x_, 0}, {var_y_, 3}, {var_z_, 3}}},
      {{{var_x_, 3}, {var_y_, 3}, {var_z_, 0}}},
      {{{var_x_, 3}, {var_y_, 0}, {var_z_, 3}}},
      {{{var_x_, 3}, {var_y_, 3}, {var_z_, 3}}}};
  for (const auto& test_env : test_envs) {
    EXPECT_NEAR((expanded - expected).Evaluate(test_env), 0.0, 1e-10);
  }
}

TEST_F(SymbolicExpressionTest, TaylorExpand4) {
  // Test TaylorExpand(7, {}, 2) = 7.
  const Expression e{7.0};
  EXPECT_PRED2(ExprEqual, e, TaylorExpand(e, Environment{}, 2));
}

TEST_F(SymbolicExpressionTest, TaylorExpandPartialEnv1) {
  // Test TaylorExpand(sin(x) + cos(y), {x:1}, 2).
  // Note that we provide a partial environment, {x:1}.
  const Expression& x{x_};
  const Expression& y{y_};
  const Expression e{sin(x) + cos(y)};
  const Environment env{{{var_x_, 1}}};
  const Expression expanded{TaylorExpand(e, env, 2)};
  // We have the following from Wolfram Alpha.
  // The query was "series sin(x) + cos(y) at x=1 to order 2".
  const Expression expected{cos(y) + sin(1) + (x - 1) * cos(1) -
                            0.5 * (x - 1) * (x - 1) * sin(1)};

  // To show that the function `expanded` approximates another function
  // `expected`, we sample a few points around x = 1 and check the evaluation
  // results over those points. For each point p, the difference between
  // expanded(p) and expected(p) should be bounded by a tiny number (here, we
  // picked 1e-10).
  const vector<Environment> test_envs{{{{var_x_, 0}, {var_y_, 0}}},
                                      {{{var_x_, 2}, {var_y_, 0}}},
                                      {{{var_x_, 0}, {var_y_, 2}}},
                                      {{{var_x_, 2}, {var_y_, 2}}}};
  for (const auto& test_env : test_envs) {
    EXPECT_NEAR((expanded - expected).Evaluate(test_env), 0.0, 1e-10);
  }
}

TEST_F(SymbolicExpressionTest, TaylorExpandPartialEnv2) {
  // Test TaylorExpand(a * sin(x), {x:2}, 3).
  // Note that we provide a partial environment, {x:2}.
  const Expression& a{a_};
  const Expression& x{x_};
  const Expression e{a * sin(x)};
  const Environment env{{{var_x_, 2}}};
  const Expression expanded{TaylorExpand(e, env, 3)};
  // We have the following from Wolfram Alpha.
  // The query was "series a * sin(x) at x=2 to order 3".
  const Expression expected{a * sin(2) + a * (x - 2) * cos(2) -
                            0.5 * (x - 2) * (x - 2) * a * sin(2) -
                            1.0 / 6.0 * pow((x - 2), 3) * a * cos(2)};

  // To show that the function `expanded` approximates another function
  // `expected`, we sample a few points around x = 2 and check the evaluation
  // results over those points. For each point p, the difference between
  // expanded(p) and expected(p) should be bounded by a tiny number (here, we
  // picked 1e-10).
  const vector<Environment> test_envs{{{{var_x_, 1}, {var_a_, 0}}},
                                      {{{var_x_, 3}, {var_a_, 0}}},
                                      {{{var_x_, 1}, {var_a_, 2}}},
                                      {{{var_x_, 3}, {var_a_, 2}}}};
  for (const auto& test_env : test_envs) {
    EXPECT_NEAR((expanded - expected).Evaluate(test_env), 0.0, 1e-10);
  }
}

// Tests std::uniform_real_distribution<drake::symbolic::Expression>.
TEST_F(SymbolicExpressionTest, UniformRealDistribution) {
  using std::uniform_real_distribution;
  {
    // Constructor with zero arguments.
    uniform_real_distribution<double> double_distribution{};
    uniform_real_distribution<Expression> symbolic_distribution{};
    EXPECT_EQ(double_distribution.a(), symbolic_distribution.a().Evaluate());
    EXPECT_EQ(double_distribution.b(), symbolic_distribution.b().Evaluate());
  }
  {
    // Constructor with a single argument.
    uniform_real_distribution<double> double_distribution{-10};
    uniform_real_distribution<Expression> symbolic_distribution{-10};
    EXPECT_EQ(double_distribution.a(), symbolic_distribution.a().Evaluate());
    EXPECT_EQ(double_distribution.b(), symbolic_distribution.b().Evaluate());
  }

  // Constructor with two arguments.
  uniform_real_distribution<double> double_distribution{-10, 10};
  uniform_real_distribution<Expression> symbolic_distribution{-10, 10};
  EXPECT_EQ(double_distribution.a(), symbolic_distribution.a().Evaluate());
  EXPECT_EQ(double_distribution.b(), symbolic_distribution.b().Evaluate());

  // Exceptions at construction.
  {
    EXPECT_THROW(uniform_real_distribution<Expression>(1.0, 0.0),
                 runtime_error);
  }

  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  // The standard case U(0, 1) should generate an expression `0.0 + (1.0 - 0.0)
  // * v` which is simplified to `v` where `v` is a random uniform variable.
  {
    uniform_real_distribution<Expression> d{0.0, 1.0};
    const Expression e{d(generator)};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_UNIFORM);
  }

  // Checks the same thing, but tests operator() with no arguments.
  {
    uniform_real_distribution<Expression> d{0.0, 1.0};
    const Expression e{d()};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_UNIFORM);
  }

  // A general case: X ~ U(-5, 10) should generate a symbolic expression
  // -5 + 15 * v where v is a random uniform variable.
  {
    uniform_real_distribution<Expression> d{-5, 10};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 1);
    const Variable& v{*(vars.begin())};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_UNIFORM);
    EXPECT_PRED2(ExprEqual, e, -5 + 15 * v);
  }

  // A general case: X ~ U(x, y) should generate a symbolic expression
  // x + (y - x) * v where v is a random uniform variable.
  {
    uniform_real_distribution<Expression> d{x_, y_};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 3);
    const auto it = find_if(vars.begin(), vars.end(), [](const Variable& v) {
      return v.get_type() == Variable::Type::RANDOM_UNIFORM;
    });
    ASSERT_TRUE(it != vars.end());
    const Variable& v{*it};
    EXPECT_PRED2(ExprEqual, e, x_ + (y_ - x_) * v);
  }

  // After reset(), it should reuse the symbolic random variables that it has
  // created.
  {
    uniform_real_distribution<Expression> d(0.0, 1.0);

    const Expression e1{d(generator)};
    const Expression e2{d(generator)};
    d.reset();
    const Expression e3{d(generator)};
    const Expression e4{d(generator)};

    EXPECT_FALSE(e1.EqualTo(e2));
    EXPECT_TRUE(e1.EqualTo(e3));
    EXPECT_TRUE(e2.EqualTo(e4));
  }

  // The two distributions show the same behavior when the same random number
  // generator is passed.
  const double value{symbolic_distribution(generator).Evaluate(&generator)};
  const double expected{double_distribution(generator_copy)};
  EXPECT_EQ(value, expected);

  // min() and max().
  EXPECT_EQ(double_distribution.min(), symbolic_distribution.min().Evaluate());
  EXPECT_EQ(double_distribution.max(), symbolic_distribution.max().Evaluate());

  // operator== and operator!=.
  {
    uniform_real_distribution<Expression> d1(0.0, 1.0);
    uniform_real_distribution<Expression> d2(d1);

    // d1 and d2 have the same parameters and the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    const Expression e1_1{d1(generator)};
    const Expression e1_2{d1(generator)};

    // The internal states of d1 has changed.
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    const Expression e2_1{d2(generator)};
    const Expression e2_2{d2(generator)};

    // Now d1 and d2 have the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that {e1_1, e1_2} and {e2_1, e2_2} are the same.
    EXPECT_TRUE(e1_1.EqualTo(e2_1));
    EXPECT_TRUE(e1_2.EqualTo(e2_2));

    // After resetting d2, d1 and d2 are not the same anymore.
    d2.reset();
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    // After resetting d1 as well, d1 and d2 are identical.
    d1.reset();
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that the two newly created distributions have the same parameters,
    // however, they are considered NOT identical.
    EXPECT_FALSE(uniform_real_distribution<Expression>(0.0, 1.0) ==
                 uniform_real_distribution<Expression>(0.0, 1.0));
    EXPECT_TRUE(uniform_real_distribution<Expression>(0.0, 1.0) !=
                uniform_real_distribution<Expression>(0.0, 1.0));
  }

  // operator<<
  ostringstream oss;
  oss << symbolic_distribution;
  EXPECT_EQ(oss.str(), "-10 10");
}

// Tests std::normal_distribution<drake::symbolic::Expression>.
TEST_F(SymbolicExpressionTest, NormalDistribution) {
  using std::normal_distribution;
  {
    // Constructor with zero arguments.
    normal_distribution<double> double_distribution{};
    normal_distribution<Expression> symbolic_distribution{};
    EXPECT_EQ(double_distribution.mean(),
              symbolic_distribution.mean().Evaluate());
    EXPECT_EQ(double_distribution.stddev(),
              symbolic_distribution.stddev().Evaluate());
  }
  {
    // Constructor with a single argument.
    normal_distribution<double> double_distribution{-10};
    normal_distribution<Expression> symbolic_distribution{-10};
    EXPECT_EQ(double_distribution.mean(),
              symbolic_distribution.mean().Evaluate());
    EXPECT_EQ(double_distribution.stddev(),
              symbolic_distribution.stddev().Evaluate());
  }

  // Constructor with two arguments.
  normal_distribution<double> double_distribution{5, 10};
  normal_distribution<Expression> symbolic_distribution{5, 10};
  EXPECT_EQ(double_distribution.mean(),
            symbolic_distribution.mean().Evaluate());
  EXPECT_EQ(double_distribution.stddev(),
            symbolic_distribution.stddev().Evaluate());

  // Exceptions at construction.
  { EXPECT_THROW(normal_distribution<Expression>(1.0, -1.0), runtime_error); }

  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  // The standard case N(0, 1) should generate an expression `0.0 + 1.0 * v`
  // which is simplified to `v` where `v` is a random Gaussian variable.
  {
    normal_distribution<Expression> d{0.0, 1.0};
    const Expression e{d(generator)};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_GAUSSIAN);
  }

  // Checks the same thing, but tests operator() with no arguments.
  {
    normal_distribution<Expression> d{0.0, 1.0};
    const Expression e{d()};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_GAUSSIAN);
  }

  // A general case: X ~ N(5, 10) should generate a symbolic expression
  // 5 + 10 * v where v is a random Gaussian variable.
  {
    normal_distribution<Expression> d{5, 10};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 1);
    const Variable& v{*(vars.begin())};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_GAUSSIAN);
    EXPECT_PRED2(ExprEqual, e, 5 + 10 * v);
  }

  // A general case: X ~ N(x, y) should generate a symbolic expression
  // x + y * v where v is a random Gaussian variable.
  {
    normal_distribution<Expression> d{x_, y_};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 3);
    const auto it = find_if(vars.begin(), vars.end(), [](const Variable& v) {
      return v.get_type() == Variable::Type::RANDOM_GAUSSIAN;
    });
    ASSERT_TRUE(it != vars.end());
    const Variable& v{*it};
    EXPECT_PRED2(ExprEqual, e, x_ + y_ * v);
  }

  // After reset(), it should reuse the symbolic random variables that it has
  // created.
  {
    normal_distribution<Expression> d(0.0, 1.0);

    const Expression e1{d(generator)};
    const Expression e2{d(generator)};
    d.reset();
    const Expression e3{d(generator)};
    const Expression e4{d(generator)};

    EXPECT_FALSE(e1.EqualTo(e2));
    EXPECT_TRUE(e1.EqualTo(e3));
    EXPECT_TRUE(e2.EqualTo(e4));
  }

  // The two distributions show the same behavior when the same random number
  // generator is passed.
  const double value{symbolic_distribution(generator).Evaluate(&generator)};
  const double expected{double_distribution(generator_copy)};
  EXPECT_EQ(value, expected);

  // min() and max().
  EXPECT_EQ(symbolic_distribution.min().Evaluate(),
            -std::numeric_limits<double>::infinity());
  EXPECT_EQ(symbolic_distribution.max().Evaluate(),
            +std::numeric_limits<double>::infinity());

  // operator== and operator!=.
  {
    normal_distribution<Expression> d1(0.0, 1.0);
    normal_distribution<Expression> d2(d1);

    // d1 and d2 have the same parameters and the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    const Expression e1_1{d1(generator)};
    const Expression e1_2{d1(generator)};

    // The internal states of d1 has changed.
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    const Expression e2_1{d2(generator)};
    const Expression e2_2{d2(generator)};

    // Now d1 and d2 have the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that {e1_1, e1_2} and {e2_1, e2_2} are the same.
    EXPECT_TRUE(e1_1.EqualTo(e2_1));
    EXPECT_TRUE(e1_2.EqualTo(e2_2));

    // After resetting d2, d1 and d2 are not the same anymore.
    d2.reset();
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    // After resetting d1 as well, d1 and d2 are identical.
    d1.reset();
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that the two newly created distributions have the same parameters,
    // however, they are considered NOT identical.
    EXPECT_FALSE(normal_distribution<Expression>(0.0, 1.0) ==
                 normal_distribution<Expression>(0.0, 1.0));
    EXPECT_TRUE(normal_distribution<Expression>(0.0, 1.0) !=
                normal_distribution<Expression>(0.0, 1.0));
  }

  // operator<<
  ostringstream oss;
  oss << symbolic_distribution;
  EXPECT_EQ(oss.str(), "5 10");
}

// Tests std::exponential_distribution<drake::symbolic::Expression>.
TEST_F(SymbolicExpressionTest, ExponentialDistribution) {
  using std::exponential_distribution;
  {
    // Constructor with zero arguments.
    exponential_distribution<double> double_distribution{};
    exponential_distribution<Expression> symbolic_distribution{};
    EXPECT_EQ(double_distribution.lambda(),
              symbolic_distribution.lambda().Evaluate());
  }

  // Constructor with a single argument.
  exponential_distribution<double> double_distribution{5.0};
  exponential_distribution<Expression> symbolic_distribution{5.0};
  EXPECT_EQ(double_distribution.lambda(),
            symbolic_distribution.lambda().Evaluate());

  // Exceptions at construction.
  { EXPECT_THROW(exponential_distribution<Expression>(-3.0), runtime_error); }

  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  // The standard case Exp(1) should generate an expression `v / 1` which is
  // simplified to `v` where `v` is a random exponential variable.
  {
    exponential_distribution<Expression> d{1.0};
    const Expression e{d(generator)};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_EXPONENTIAL);
  }

  // Checks the same thing, but tests operator() with no arguments.
  {
    exponential_distribution<Expression> d{1.0};
    const Expression e{d()};
    ASSERT_TRUE(is_variable(e));
    const Variable& v{get_variable(e)};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_EXPONENTIAL);
  }

  // A general case: X ~ Exp(5) should generate a symbolic expression
  // v / 5 where v is a random exponential variable.
  {
    exponential_distribution<Expression> d{5};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 1);
    const Variable& v{*(vars.begin())};
    EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_EXPONENTIAL);
    EXPECT_PRED2(ExprEqual, e, v / 5);
  }

  // A general case: X ~ EXP(y) should generate a symbolic expression
  // v / y where v is a random exponential variable.
  {
    exponential_distribution<Expression> d{y_};
    const Expression e{d(generator)};
    const Variables vars{e.GetVariables()};
    ASSERT_EQ(vars.size(), 2);
    const auto it = find_if(vars.begin(), vars.end(), [](const Variable& v) {
      return v.get_type() == Variable::Type::RANDOM_EXPONENTIAL;
    });
    ASSERT_TRUE(it != vars.end());
    const Variable& v{*it};
    EXPECT_PRED2(ExprEqual, e, v / y_);
  }

  // After reset(), it should reuse the symbolic random variables that it has
  // created.
  {
    exponential_distribution<Expression> d(1.0);

    const Expression e1{d(generator)};
    const Expression e2{d(generator)};
    d.reset();
    const Expression e3{d(generator)};
    const Expression e4{d(generator)};

    EXPECT_FALSE(e1.EqualTo(e2));
    EXPECT_TRUE(e1.EqualTo(e3));
    EXPECT_TRUE(e2.EqualTo(e4));
  }

  // The two distributions show the same behavior when the same random number
  // generator is passed.
  const double value{symbolic_distribution(generator).Evaluate(&generator)};
  const double expected{double_distribution(generator_copy)};
  EXPECT_EQ(value, expected);

  // min() and max().
  EXPECT_EQ(symbolic_distribution.min().Evaluate(), 0.0);
  EXPECT_EQ(symbolic_distribution.max().Evaluate(),
            +std::numeric_limits<double>::infinity());

  // operator== and operator!=.
  {
    exponential_distribution<Expression> d1(2.0);
    exponential_distribution<Expression> d2(d1);

    // d1 and d2 have the same parameters and the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    const Expression e1_1{d1(generator)};
    const Expression e1_2{d1(generator)};

    // The internal states of d1 has changed.
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    const Expression e2_1{d2(generator)};
    const Expression e2_2{d2(generator)};

    // Now d1 and d2 have the same internal states.
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that {e1_1, e1_2} and {e2_1, e2_2} are the same.
    EXPECT_TRUE(e1_1.EqualTo(e2_1));
    EXPECT_TRUE(e1_2.EqualTo(e2_2));

    // After resetting d2, d1 and d2 are not the same anymore.
    d2.reset();
    EXPECT_FALSE(d1 == d2);
    EXPECT_TRUE(d1 != d2);

    // After resetting d1 as well, d1 and d2 are identical.
    d1.reset();
    EXPECT_TRUE(d1 == d2);
    EXPECT_FALSE(d1 != d2);

    // Note that the two newly created distributions have the same parameters,
    // however, they are considered NOT identical.
    EXPECT_FALSE(exponential_distribution<Expression>(2.0) ==
                 exponential_distribution<Expression>(2.0));
    EXPECT_TRUE(exponential_distribution<Expression>(2.0) !=
                exponential_distribution<Expression>(2.0));
  }

  // operator<<
  ostringstream oss;
  oss << symbolic_distribution;
  EXPECT_EQ(oss.str(), "5");
}

// This function checks if the following commute diagram works for given a
// symbolic expression `e`, a symbolic environment `env`, and a `random
// generator`.
//
//                         Substitute Random Variables
// +---------------------+     with Sampled Values     +--------------------+
// |     Expression      |                             |    Expression      |
// |with Random Variables+---------------------------->+w/o Random Variables|
// +----------+----------+                             +---------+----------+
//            |                                                  |
//            v                                                  v
//        Evaluate                                            Evaluate
// with a Random Generator                             w/o a Random Generator
//            +                                                  +
//            |                                                  |
//            v                                                  v
// +----------+----------+                             +---------+----------+
// |    double value     +-----------  ==  ------------+   double value     |
// +---------------------+                             +--------------------+
::testing::AssertionResult CheckExpressionWithRandomVariables(
    const Expression& e, const Environment& env,
    RandomGenerator* const random_generator) {
  RandomGenerator random_generator_copy(*random_generator);
  const double v1{e.Evaluate(env, random_generator)};

  const Environment env_extended{
      PopulateRandomVariables(env, e.GetVariables(), &random_generator_copy)};
  const double v2{e.Evaluate(env_extended, nullptr)};

  if (v1 == v2) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "Different evaluation results:\n"
           << "e = " << e << "\n"
           << "env = " << env << "\n"
           << "env_extended = " << env_extended << "\n"
           << "v1 = " << v1 << " and v2 = " << v2;
  }
}

TEST_F(SymbolicExpressionTest, EvaluateExpressionsIncludingRandomVariables) {
  const Variable uni1{"uniform1", Variable::Type::RANDOM_UNIFORM};
  const Variable uni2{"uniform2", Variable::Type::RANDOM_UNIFORM};
  const Variable gau1{"gaussian1", Variable::Type::RANDOM_GAUSSIAN};
  const Variable gau2{"gaussian2", Variable::Type::RANDOM_GAUSSIAN};
  const Variable exp1{"exponential1", Variable::Type::RANDOM_EXPONENTIAL};
  const Variable exp2{"exponential2", Variable::Type::RANDOM_EXPONENTIAL};

  const vector<Expression> expressions{
      uni1 * uni2,
      gau1 * gau2,
      exp1 * exp2,
      x_ * sin(uni1) * cos(uni1) * tan(uni1) + y_,
      exp1 * pow(abs(x_), exp1) + exp2,
      x_ / (1 + exp(abs(gau1 * gau2))) + y_ * pow(exp1, 4 + y_),
      x_ * uni1 + y_ * uni1 + x_ * gau1 + y_ * gau1 + x_ * exp1 + y_ * exp1,
  };

  const vector<Environment> environments{
      {{{var_x_, -1.0}, {var_y_, 3.0}}},
      {{{var_x_, 1.0}, {var_y_, 1.0}}},
      {{{var_x_, 2.0}, {var_y_, 1.0}}},
      {{{var_x_, 2.0}, {var_y_, -2.0}}},
  };

  RandomGenerator generator{};
  for (const Expression& e : expressions) {
    for (const Environment& env : environments) {
      EXPECT_TRUE(CheckExpressionWithRandomVariables(e, env, &generator));
    }
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
