#define DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER
#include "drake/common/symbolic_expression_cell.h"
#undef DRAKE_COMMON_SYMBOLIC_DETAIL_HEADER

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;
using test::FormulaEqual;

// Provides common variables and expressions that are used by the following
// tests.
class SymbolicExpressionCellTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Expression x_{var_x_};
  const Expression y_{var_y_};

  const Expression e_constant_{1.0};
  const Expression e_var_{var_x_};
  const Expression e_add_{x_ + y_};
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
  const Expression e_uf_{uninterpreted_function("uf", {var_x_, var_y_})};
};

TEST_F(SymbolicExpressionCellTest, CastFunctionsConst) {
  EXPECT_EQ(to_constant(e_constant_).get_value(),
            get_constant_value(e_constant_));
  EXPECT_EQ(to_variable(e_var_).get_variable(), get_variable(e_var_));

  EXPECT_EQ(to_addition(e_add_).get_constant(),
            get_constant_in_addition(e_add_));
  EXPECT_EQ(to_addition(e_add_).get_expr_to_coeff_map(),
            get_expr_to_coeff_map_in_addition(e_add_));

  EXPECT_EQ(to_multiplication(e_mul_).get_constant(),
            get_constant_in_multiplication(e_mul_));
  EXPECT_EQ(to_multiplication(e_mul_).get_base_to_exponent_map().size(),
            get_base_to_exponent_map_in_multiplication(e_mul_).size());
  EXPECT_EQ(to_multiplication(e_mul_).get_base_to_exponent_map().size(), 2);
  EXPECT_PRED2(ExprEqual,
               to_multiplication(e_mul_).get_base_to_exponent_map().at(var_x_),
               get_base_to_exponent_map_in_multiplication(e_mul_).at(var_x_));
  EXPECT_PRED2(ExprEqual,
               to_multiplication(e_mul_).get_base_to_exponent_map().at(var_y_),
               get_base_to_exponent_map_in_multiplication(e_mul_).at(var_y_));

  EXPECT_EQ(to_division(e_div_).get_second_argument(),
            get_second_argument(e_div_));
  EXPECT_EQ(to_division(e_div_).get_second_argument(),
            get_second_argument(e_div_));

  EXPECT_EQ(to_log(e_log_).get_argument(), get_argument(e_log_));
  EXPECT_EQ(to_abs(e_abs_).get_argument(), get_argument(e_abs_));
  EXPECT_EQ(to_exp(e_exp_).get_argument(), get_argument(e_exp_));
  EXPECT_EQ(to_sqrt(e_sqrt_).get_argument(), get_argument(e_sqrt_));
  EXPECT_EQ(to_pow(e_pow_).get_first_argument(), get_first_argument(e_pow_));
  EXPECT_EQ(to_pow(e_pow_).get_second_argument(), get_second_argument(e_pow_));
  EXPECT_EQ(to_sin(e_sin_).get_argument(), get_argument(e_sin_));
  EXPECT_EQ(to_cos(e_cos_).get_argument(), get_argument(e_cos_));
  EXPECT_EQ(to_tan(e_tan_).get_argument(), get_argument(e_tan_));
  EXPECT_EQ(to_asin(e_asin_).get_argument(), get_argument(e_asin_));
  EXPECT_EQ(to_acos(e_acos_).get_argument(), get_argument(e_acos_));
  EXPECT_EQ(to_atan(e_atan_).get_argument(), get_argument(e_atan_));
  EXPECT_EQ(to_atan2(e_atan2_).get_first_argument(),
            get_first_argument(e_atan2_));
  EXPECT_EQ(to_atan2(e_atan2_).get_second_argument(),
            get_second_argument(e_atan2_));
  EXPECT_EQ(to_sinh(e_sinh_).get_argument(), get_argument(e_sinh_));
  EXPECT_EQ(to_cosh(e_cosh_).get_argument(), get_argument(e_cosh_));
  EXPECT_EQ(to_tanh(e_tanh_).get_argument(), get_argument(e_tanh_));
  EXPECT_EQ(to_min(e_min_).get_first_argument(), get_first_argument(e_min_));
  EXPECT_EQ(to_min(e_min_).get_second_argument(), get_second_argument(e_min_));
  EXPECT_EQ(to_max(e_max_).get_first_argument(), get_first_argument(e_max_));
  EXPECT_EQ(to_max(e_max_).get_second_argument(), get_second_argument(e_max_));
  EXPECT_EQ(to_ceil(e_ceil_).get_argument(), get_argument(e_ceil_));
  EXPECT_EQ(to_floor(e_floor_).get_argument(), get_argument(e_floor_));
  EXPECT_PRED2(FormulaEqual, to_if_then_else(e_ite_).get_conditional_formula(),
               get_conditional_formula(e_ite_));
  EXPECT_PRED2(ExprEqual, to_if_then_else(e_ite_).get_then_expression(),
               get_then_expression(e_ite_));
  EXPECT_PRED2(ExprEqual, to_if_then_else(e_ite_).get_else_expression(),
               get_else_expression(e_ite_));
  EXPECT_EQ(to_uninterpreted_function(e_uf_).get_name(),
            get_uninterpreted_function_name(e_uf_));
  EXPECT_EQ(to_uninterpreted_function(e_uf_).get_arguments().size(), 2);
  EXPECT_EQ(to_uninterpreted_function(e_uf_).get_arguments().size(),
            get_uninterpreted_function_arguments(e_uf_).size());
  EXPECT_EQ(to_uninterpreted_function(e_uf_).get_arguments()[0],
            get_uninterpreted_function_arguments(e_uf_)[0]);
  EXPECT_EQ(to_uninterpreted_function(e_uf_).get_arguments()[1],
            get_uninterpreted_function_arguments(e_uf_)[1]);
}

TEST_F(SymbolicExpressionCellTest, CastFunctionsNonConst) {
  EXPECT_EQ(to_constant(Expression{e_constant_}).get_value(),
            get_constant_value(e_constant_));
  EXPECT_EQ(to_variable(Expression{e_var_}).get_variable(),
            get_variable(e_var_));

  EXPECT_EQ(to_addition(Expression{e_add_}).get_constant(),
            get_constant_in_addition(e_add_));
  EXPECT_EQ(to_addition(Expression{e_add_}).get_expr_to_coeff_map(),
            get_expr_to_coeff_map_in_addition(e_add_));

  EXPECT_EQ(to_multiplication(Expression{e_mul_}).get_constant(),
            get_constant_in_multiplication(e_mul_));
  EXPECT_EQ(
      to_multiplication(Expression{e_mul_}).get_base_to_exponent_map().size(),
      get_base_to_exponent_map_in_multiplication(e_mul_).size());
  EXPECT_EQ(
      to_multiplication(Expression{e_mul_}).get_base_to_exponent_map().size(),
      2);
  EXPECT_PRED2(ExprEqual,
               to_multiplication(Expression{e_mul_})
                   .get_base_to_exponent_map()
                   .at(var_x_),
               get_base_to_exponent_map_in_multiplication(e_mul_).at(var_x_));
  EXPECT_PRED2(ExprEqual,
               to_multiplication(Expression{e_mul_})
                   .get_base_to_exponent_map()
                   .at(var_y_),
               get_base_to_exponent_map_in_multiplication(e_mul_).at(var_y_));

  EXPECT_EQ(to_division(Expression{e_div_}).get_second_argument(),
            get_second_argument(e_div_));
  EXPECT_EQ(to_division(Expression{e_div_}).get_second_argument(),
            get_second_argument(e_div_));

  EXPECT_EQ(to_log(Expression{e_log_}).get_argument(), get_argument(e_log_));
  EXPECT_EQ(to_abs(Expression{e_abs_}).get_argument(), get_argument(e_abs_));
  EXPECT_EQ(to_exp(Expression{e_exp_}).get_argument(), get_argument(e_exp_));
  EXPECT_EQ(to_sqrt(Expression{e_sqrt_}).get_argument(),
            get_argument(e_sqrt_));
  EXPECT_EQ(to_pow(Expression{e_pow_}).get_first_argument(),
            get_first_argument(e_pow_));
  EXPECT_EQ(to_pow(Expression{e_pow_}).get_second_argument(),
            get_second_argument(e_pow_));
  EXPECT_EQ(to_sin(Expression{e_sin_}).get_argument(), get_argument(e_sin_));
  EXPECT_EQ(to_cos(Expression{e_cos_}).get_argument(), get_argument(e_cos_));
  EXPECT_EQ(to_tan(Expression{e_tan_}).get_argument(), get_argument(e_tan_));
  EXPECT_EQ(to_asin(Expression{e_asin_}).get_argument(),
            get_argument(e_asin_));
  EXPECT_EQ(to_acos(Expression{e_acos_}).get_argument(),
            get_argument(e_acos_));
  EXPECT_EQ(to_atan(Expression{e_atan_}).get_argument(),
            get_argument(e_atan_));
  EXPECT_EQ(to_atan2(Expression{e_atan2_}).get_first_argument(),
            get_first_argument(e_atan2_));
  EXPECT_EQ(to_atan2(Expression{e_atan2_}).get_second_argument(),
            get_second_argument(e_atan2_));
  EXPECT_EQ(to_sinh(Expression{e_sinh_}).get_argument(),
            get_argument(e_sinh_));
  EXPECT_EQ(to_cosh(Expression{e_cosh_}).get_argument(),
            get_argument(e_cosh_));
  EXPECT_EQ(to_tanh(Expression{e_tanh_}).get_argument(),
            get_argument(e_tanh_));
  EXPECT_EQ(to_min(Expression{e_min_}).get_first_argument(),
            get_first_argument(e_min_));
  EXPECT_EQ(to_min(Expression{e_min_}).get_second_argument(),
            get_second_argument(e_min_));
  EXPECT_EQ(to_max(Expression{e_max_}).get_first_argument(),
            get_first_argument(e_max_));
  EXPECT_EQ(to_max(Expression{e_max_}).get_second_argument(),
            get_second_argument(e_max_));
  EXPECT_EQ(to_ceil(Expression{e_ceil_}).get_argument(),
            get_argument(e_ceil_));
  EXPECT_EQ(to_floor(Expression{e_floor_}).get_argument(),
            get_argument(e_floor_));
  EXPECT_PRED2(FormulaEqual,
               to_if_then_else(Expression{e_ite_}).get_conditional_formula(),
               get_conditional_formula(e_ite_));
  EXPECT_PRED2(ExprEqual,
               to_if_then_else(Expression{e_ite_}).get_then_expression(),
               get_then_expression(e_ite_));
  EXPECT_PRED2(ExprEqual,
               to_if_then_else(Expression{e_ite_}).get_else_expression(),
               get_else_expression(e_ite_));
  EXPECT_EQ(to_uninterpreted_function(Expression{e_uf_}).get_name(),
            get_uninterpreted_function_name(e_uf_));
  EXPECT_EQ(
      to_uninterpreted_function(Expression{e_uf_}).get_arguments().size(), 2);
  EXPECT_EQ(
      to_uninterpreted_function(Expression{e_uf_}).get_arguments().size(),
      get_uninterpreted_function_arguments(e_uf_).size());
  EXPECT_EQ(to_uninterpreted_function(Expression{e_uf_}).get_arguments()[0],
            get_uninterpreted_function_arguments(e_uf_)[0]);
  EXPECT_EQ(to_uninterpreted_function(Expression{e_uf_}).get_arguments()[1],
            get_uninterpreted_function_arguments(e_uf_)[1]);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
