#include "drake/common/symbolic_expression.h"

#include <cmath>
#include <exception>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::domain_error;
using std::exception;
using std::get;
using std::tuple;
using std::vector;

using test::ExprEqual;

class SymbolicDifferentiationTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
};

// Given a function f and a variable x_i, returns newton's difference quotient
// of f :
//
// f(x1, ..., x_i + delta, ..., x_n)  - f(x_1, ..., x_i, ..., x_n)
// ---------------------------------------------------------------
//                          delta
//
// where delta = sqrt(2.2e-16) * env[x_i]. See "Numerical Recipes in C", Chapter
// 5.7 to find more details about this choice of delta.
//
// We use this expression to check the correctness of Expression::Differentiate
// method numerically.
Expression DifferenceQuotient(const Expression& f, const Variable& x,
                              const Environment& env) {
  const double delta{std::sqrt(2.2e-16) * Expression{x}.Evaluate(env)};
  return (f.Substitute(x, x + delta) - f) / delta;
}

TEST_F(SymbolicDifferentiationTest, SymbolicBaseCases) {
  const Expression& x{var_x_};
  const Expression& y{var_y_};
  const Expression& z{var_z_};

  vector<tuple<Expression, Variable, Expression>> check_list;
  // ∂/∂x (0) = 0
  check_list.emplace_back(0.0, var_x_, 0);
  // ∂/∂x (1) = 0
  check_list.emplace_back(1.0, var_x_, 0);
  // ∂/∂x (x) = 1
  check_list.emplace_back(x, var_x_, 1);
  // ∂/∂x (2 + x + y + z) = 1
  check_list.emplace_back(2 + x + y + z, var_x_, 1);
  // ∂/∂x (2 + 3*x) = 3
  check_list.emplace_back(2 + 3 * x, var_x_, 3);
  // ∂/∂x (1/x) = -1 / x^2
  check_list.emplace_back(1 / x, var_x_, -1 / pow(x, 2));
  // ∂/∂x log(x) = 1 / x
  check_list.emplace_back(log(x), var_x_, 1 / x);
  // ∂/∂x exp(x) = exp(x)
  check_list.emplace_back(exp(x), var_x_, exp(x));
  // ∂/∂x sqrt(x) = 1 / (2 * sqrt(x))
  check_list.emplace_back(sqrt(x), var_x_, 1 / (2 * sqrt(x)));
  // ∂/∂x pow(x, 5) = 5 * pow(x, 4)
  check_list.emplace_back(pow(x, 5), var_x_, 5 * pow(x, 4));
  // ∂/∂x pow(5, x) = pow(5, x) * log(5)
  check_list.emplace_back(pow(5, x), var_x_, pow(5, x) * log(5));
  // ∂/∂x pow(x, y) = y * pow(x, y - 1)
  check_list.emplace_back(pow(x, y), var_x_, y * pow(x, y - 1));
  // ∂/∂y pow(x, y) = pow(x, y) * log(x)
  check_list.emplace_back(pow(x, y), var_y_, pow(x, y) * log(x));
  // ∂/∂x sin(x) = cos(x)
  check_list.emplace_back(sin(x), var_x_, cos(x));
  // ∂/∂x cos(x) = -sin(x)
  check_list.emplace_back(cos(x), var_x_, -sin(x));
  // ∂/∂x tan(x) = sec^2(x) = 1 / cos(x)^2
  check_list.emplace_back(tan(x), var_x_, 1 / (cos(x) * cos(x)));
  // ∂/∂x asin(x) = 1 / (sqrt(1-x^2))
  check_list.emplace_back(asin(x), var_x_, 1 / (sqrt(1 - pow(x, 2))));
  // ∂/∂x acos(x) = -1 / (sqrt(1-x^2))
  check_list.emplace_back(acos(x), var_x_, -1 / (sqrt(1 - pow(x, 2))));
  // ∂/∂x atan(x) = 1 / (x^2 + 1)
  check_list.emplace_back(atan(x), var_x_, 1 / (pow(x, 2) + 1));
  // ∂/∂x atan2(x, y) = y / (x^2 + y^2)
  check_list.emplace_back(atan2(x, y), var_x_, y / (pow(x, 2) + pow(y, 2)));
  // ∂/∂y atan2(x, y) = -x / (x^2 + y^2)
  check_list.emplace_back(atan2(x, y), var_y_, -x / (pow(x, 2) + pow(y, 2)));
  // ∂/∂x sinh x = cosh(x)
  check_list.emplace_back(sinh(x), var_x_, cosh(x));
  // ∂/∂x cosh(x) = sinh(x)
  check_list.emplace_back(cosh(x), var_x_, sinh(x));
  // ∂/∂x tanh(x) = 1 / (pow(cosh(x), 2))
  check_list.emplace_back(tanh(x), var_x_, 1 / (pow(cosh(x), 2)));
  for (const tuple<Expression, Variable, Expression>& item : check_list) {
    const Expression& expr{get<0>(item)};
    const Variable& var{get<1>(item)};
    const Expression& diff_result{get<2>(item)};
    EXPECT_PRED2(ExprEqual, expr.Differentiate(var), diff_result);
  }
}

TEST_F(SymbolicDifferentiationTest, SymbolicCompoundCases) {
  const Expression& x{var_x_};
  const Expression& y{var_y_};
  const Expression& z{var_z_};
  vector<tuple<Expression, Variable, Expression>> check_list;
  // ∂/∂x (5 + x + 2 * x^2 + 3 * x^3) = 1 + 4x + 9x^2
  check_list.emplace_back(5 + x + 2 * x * x + 3 * pow(x, 3), var_x_,
                          1 + 4 * x + 9 * x * x);
  // ∂/∂x (-2 - 3 * x + 6 * x^2 - 4 * x^5) = -3 + 12 x - 20 x^4
  check_list.emplace_back(-2 - 3 * x + 6 * pow(x, 2) - 4 * pow(x, 5), var_x_,
                          -3 + 12 * x - 20 * pow(x, 4));
  // ∂/∂x (3 * (x^3 -x)^3) = 9 * (x^3 -x)^2 * (3x^2 -1)
  check_list.emplace_back(3 * pow(pow(x, 3) - x, 3), var_x_,
                          9 * pow(pow(x, 3) - x, 2) * (3 * pow(x, 2) - 1));
  // ∂/∂x [(x^3 + 2) / (x^2 - 3)]
  // = [(x^2 - 3) * 3x^2 - (x^3 + 2) * 2x] / (x^2 - 3)^2
  check_list.emplace_back(
      (pow(x, 3) + 2) / (pow(x, 2) - 3), var_x_,
      ((pow(x, 2) - 3) * 3 * pow(x, 2) - (pow(x, 3) + 2) * 2 * x) /
          pow(pow(x, 2) - 3, 2));
  // ∂/∂x x^3yz + xy + z + 3 = 3x^2yz + y
  check_list.emplace_back(pow(x, 3) * y * z + x * y + z + 3, var_x_,
                          3 * pow(x, 2) * y * z + y);
  // ∂/∂y x^3yz + xy + z + 3 = x^3z + x
  check_list.emplace_back(pow(x, 3) * y * z + x * y + z + 3, var_y_,
                          pow(x, 3) * z + x);
  // ∂/∂z x^3yz + xy + z + 3 = 3x^2y + 1
  check_list.emplace_back(pow(x, 3) * y * z + x * y + z + 3, var_z_,
                          pow(x, 3) * y + 1);
  // ∂/∂x sin(xy) + cos(x) = y * cos(xy) - sin(x)
  check_list.emplace_back(sin(x * y) + cos(x), var_x_, y * cos(x * y) - sin(x));
  // ∂/∂y sin(xy) + cos(x) = x * cos(xy)
  check_list.emplace_back(sin(x * y) + cos(x), var_y_, x * cos(x * y));
  // ∂/∂x x * exp(xy) = exp(xy) + xy * exp(xy)
  check_list.emplace_back(x * exp(x * y), var_x_,
                          exp(x * y) + (x * y) * exp(x * y));
  // ∂/∂y x * exp(xy) = x^2 exp(xy)
  check_list.emplace_back(x * exp(x * y), var_y_, pow(x, 2) * exp(x * y));
  // ∂/∂x log(x^2 + 2y) = 2x / (x^2 + y)
  check_list.emplace_back(log(pow(x, 2) + 2 * y), var_x_,
                          2 * x / (pow(x, 2) + 2 * y));
  // ∂/∂x log(x^2 + 2y) = 2 / (x^2 + 2 * y)
  check_list.emplace_back(log(pow(x, 2) + 2 * y), var_y_,
                          2 / (pow(x, 2) + 2 * y));
  // ∂/∂x x * sin(x - y) = x * cos(x - y) + sin(x - y)
  check_list.emplace_back(x * sin(x - y), var_x_, x * cos(x - y) + sin(x - y));
  // ∂/∂y x * sin(x - y) = -x * cos(x - y)
  check_list.emplace_back(x * sin(x - y), var_y_, -x * cos(x - y));
  // ∂/∂x sin(y * cos(x)) = -y sin(x) cos(y cos(x))
  check_list.emplace_back(sin(y * cos(x)), var_x_,
                          -y * sin(x) * cos(y * cos(x)));
  // ∂/∂y sin(y * cos(x)) = cos(x) cos(y cos(x))
  check_list.emplace_back(sin(y * cos(x)), var_y_, cos(x) * cos(y * cos(x)));
  // ∂/∂x tan(x + y^2) = 1 / cos^2(x + y^2)
  check_list.emplace_back(tan(x + pow(y, 2)), var_x_,
                          1 / (pow(cos(x + pow(y, 2)), 2)));
  // ∂/∂y tan(x + y^2) = 2y / cos^2(x + y^2)
  check_list.emplace_back(tan(x + pow(y, 2)), var_y_,
                          2 * y / (pow(cos(x + pow(y, 2)), 2)));
  // ∂/∂x asin(x + 2y) = 1 / sqrt(1 - (x + y)^2)
  check_list.emplace_back(asin(x + 2 * y), var_x_,
                          1 / sqrt(1 - pow(x + 2 * y, 2)));
  // ∂/∂y asin(x + 2y) = 2 / sqrt(1 - (x + y)^2)
  check_list.emplace_back(asin(x + 2 * y), var_y_,
                          2 / sqrt(1 - pow(x + 2 * y, 2)));
  // ∂/∂x atan(3x + 2y) = 3 / ((3x + 2y)^2 + 1)
  check_list.emplace_back(atan(3 * x + 2 * y), var_x_,
                          3 / (pow(3 * x + 2 * y, 2) + 1));
  // ∂/∂y atan(3x + 2y) = 2 / ((3x + 2y)^2 + 1)
  check_list.emplace_back(atan(3 * x + 2 * y), var_y_,
                          2 / (pow(3 * x + 2 * y, 2) + 1));
  // ∂/∂x atan2(2x, 3y) = 6y / (4x^2 + 9y^2)
  check_list.emplace_back(atan2(2 * x, 3 * y), var_x_,
                          6 * y / (pow(2 * x, 2) + pow(3 * y, 2)));
  // ∂/∂y atan2(2x, 3y) = -6x / (4x^2 + 9y^2)
  check_list.emplace_back(atan2(2 * x, 3 * y), var_y_,
                          -6 * x / (pow(2 * x, 2) + pow(3 * y, 2)));
  // ∂/∂x sinh(2x + 3y) = 2 cosh(2x + 3y)
  check_list.emplace_back(sinh(2 * x + 3 * y), var_x_, 2 * cosh(2 * x + 3 * y));
  // ∂/∂y sinh(2x + 3y) = 3 cosh(2x + 3y)
  check_list.emplace_back(sinh(2 * x + 3 * y), var_y_, 3 * cosh(2 * x + 3 * y));
  // ∂/∂x cosh(2x + 3y) = 2 sinh(2x + 3y)
  check_list.emplace_back(cosh(2 * x + 3 * y), var_x_, 2 * sinh(2 * x + 3 * y));
  // ∂/∂y cosh(2x + 3y) = 2 sinh(2x + 3y)
  check_list.emplace_back(cosh(2 * x + 3 * y), var_y_, 3 * sinh(2 * x + 3 * y));
  // ∂/∂x tanh(2x + 3y) = 2 / (cosh^2(2x + 3y))
  check_list.emplace_back(tanh(2 * x + 3 * y), var_x_,
                          2 / (pow(cosh(2 * x + 3 * y), 2)));
  // ∂/∂y tanh(2x + 3y) = 3 / (cosh^2(2x + 3y))
  check_list.emplace_back(tanh(2 * x + 3 * y), var_y_,
                          3 / (pow(cosh(2 * x + 3 * y), 2)));

  for (const tuple<Expression, Variable, Expression>& item : check_list) {
    const Expression& expr{get<0>(item)};
    const Variable& var{get<1>(item)};
    const Expression& diff_result{get<2>(item)};
    EXPECT_PRED2(ExprEqual, expr.Differentiate(var), diff_result);
  }
}

TEST_F(SymbolicDifferentiationTest, NotDifferentiable) {
  const Expression& x{var_x_};
  const Expression& y{var_y_};
  // abs, min, max, if_then_else are not differentiable with respect to a
  // variable if an argument includes the variable.
  EXPECT_ANY_THROW(abs(x).Differentiate(var_x_));
  EXPECT_ANY_THROW(min(x, y).Differentiate(var_x_));
  EXPECT_ANY_THROW(min(x, y).Differentiate(var_y_));
  EXPECT_ANY_THROW(max(x, y).Differentiate(var_x_));
  EXPECT_ANY_THROW(max(x, y).Differentiate(var_y_));
  EXPECT_ANY_THROW(if_then_else(x > y, x, y).Differentiate(var_x_));
  EXPECT_ANY_THROW(if_then_else(x > y, x, y).Differentiate(var_y_));
  // However, those functions are still differentiable if the variable for
  // differentiation does not occur in the function body.
  EXPECT_EQ(abs(x).Differentiate(var_y_), 0.0);
  EXPECT_EQ(min(x, y).Differentiate(var_z_), 0.0);
  EXPECT_EQ(max(x, y).Differentiate(var_z_), 0.0);
  EXPECT_EQ(if_then_else(x > y, x, y).Differentiate(var_z_), 0.0);
}

// It tests if the evaluation result of a symbolic differentiation is close to
// the evaluation of newton's difference quotient. Note that this test,
// NumericalTests1, only includes functions which are well-defined over Real.
TEST_F(SymbolicDifferentiationTest, NumericalTests1) {
  const Expression& x{var_x_};
  const Expression& y{var_y_};
  const Expression& z{var_z_};
  // Set up functions to test
  vector<Expression> fns;
  fns.push_back(3.0);
  fns.push_back(7 + -3 * x + 5 * y - 7 * z);
  fns.push_back(-2.0 + 2 * x * y - 3 * y * z + 0.5 * z * x);
  fns.push_back(x / y);
  fns.push_back(y / x);
  fns.push_back(1 / x * y * z);
  fns.push_back(exp(x));
  fns.push_back(sin(x));
  fns.push_back(cos(x));
  fns.push_back(tan(x));
  fns.push_back(atan(x));
  fns.push_back(atan2(x, y));
  fns.push_back(atan2(y, x));
  fns.push_back(sinh(x));
  fns.push_back(cosh(x));
  fns.push_back(tanh(x));
  fns.push_back(exp(cos(x) / 10 + tan(y) / 10));
  fns.push_back(sin(x * y + z));
  fns.push_back(cos(x / y + z));
  fns.push_back(tan(x * z - y));
  fns.push_back(atan(0.1 * (x + y - z)));
  fns.push_back(atan2(x * y, y * z));
  fns.push_back(atan2(y * z, x * z));
  fns.push_back(sinh((x * x + y * y + z * z) / 100));
  fns.push_back(cosh((x * x + y * y + z * z) / 100));
  fns.push_back(tanh((x * x + y * y + z * z) / 100));
  // Set up environments to test
  vector<Environment> envs;
  envs.push_back({{var_x_, 0.3}, {var_y_, 0.2}, {var_z_, 0.2}});
  envs.push_back({{var_x_, 1.4}, {var_y_, -0.1}, {var_z_, 3.1}});
  envs.push_back({{var_x_, 2.2}, {var_y_, 3.0}, {var_z_, 2.3}});
  envs.push_back({{var_x_, -5.7}, {var_y_, 4.9}, {var_z_, -5.4}});
  envs.push_back({{var_x_, 3.1}, {var_y_, -0.8}, {var_z_, 2.5}});
  envs.push_back({{var_x_, 2.8}, {var_y_, -5.7}, {var_z_, 3.6}});
  // Set up variables for differentiation to test
  const Variables vars{var_x_, var_y_, var_z_};
  const double eps{1e-5};  // This choice of eps is arbitrary.
  for (const Expression& f : fns) {
    for (const Environment& env : envs) {
      for (const Variable& var : vars) {
        const double grad{f.Differentiate(var).Evaluate(env)};
        const double diff{DifferenceQuotient(f, var, env).Evaluate(env)};
        EXPECT_NEAR(grad, diff, eps);
      }
    }
  }
}

// It tests if the evaluation result of a symbolic differentiation is close to
// the evaluation of newton's difference quotient. Note that this test,
// NumericalTests2, only includes functions whose domains are restricted. We've
// tailored the environments so that no domain_error occurs during the test.
TEST_F(SymbolicDifferentiationTest, NumericalTests2) {
  const Expression& x{var_x_};
  const Expression& y{var_y_};
  const Expression& z{var_z_};
  // Set up functions to test
  vector<Expression> fns;
  fns.push_back(log(x));  //
  fns.push_back(log(sin(x) + cos(x) + 2));
  fns.push_back(sqrt(x));  //
  fns.push_back(pow(x, y));
  fns.push_back(pow(y, x));
  fns.push_back(asin(x / 10));
  fns.push_back(acos(x / 10));
  fns.push_back(asin(0.1 * (x + y - z)));
  fns.push_back(acos(0.1 * (x + y - z)));
  fns.push_back(pow(x + y, y / x / 10));
  fns.push_back(pow(y * z, (x + z) / 5));
  // Set up environments to test
  vector<Environment> envs;
  envs.push_back({{var_x_, 0.3}, {var_y_, 1}, {var_z_, 0.2}});
  envs.push_back({{var_x_, 1.4}, {var_y_, 2}, {var_z_, 3.1}});
  envs.push_back({{var_x_, 2.2}, {var_y_, 4}, {var_z_, 2.3}});
  envs.push_back({{var_x_, 4.7}, {var_y_, 3}, {var_z_, 3.4}});
  envs.push_back({{var_x_, 3.1}, {var_y_, 3}, {var_z_, 2.5}});
  envs.push_back({{var_x_, 2.8}, {var_y_, 2}, {var_z_, 2.6}});
  // Set up variables for differentiation to test
  const Variables vars{var_x_, var_y_, var_z_};
  const double eps{1e-5};  // This choice of eps is arbitrary.
  for (const Expression& f : fns) {
    for (const Environment& env : envs) {
      for (const Variable& var : vars) {
        const double grad{f.Differentiate(var).Evaluate(env)};
        const double diff{DifferenceQuotient(f, var, env).Evaluate(env)};
        EXPECT_NEAR(grad, diff, eps);
      }
    }
  }
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
