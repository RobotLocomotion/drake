#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using std::function;
using std::pair;
using std::runtime_error;
using std::vector;

namespace drake {

using test::LimitMalloc;

namespace symbolic {
namespace {

using test::ExprEqual;
using test::ExprNotEqual;

class SymbolicExpansionTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  vector<Environment> envs_;

  void SetUp() override {
    // Set up environments (envs_).
    envs_.push_back({{var_x_, 1.7}, {var_y_, 2}, {var_z_, 2.3}});     // + + +
    envs_.push_back({{var_x_, -0.3}, {var_y_, 1}, {var_z_, 0.2}});    // - + +
    envs_.push_back({{var_x_, 1.4}, {var_y_, -2}, {var_z_, 3.1}});    // + - +
    envs_.push_back({{var_x_, 2.2}, {var_y_, 4}, {var_z_, -2.3}});    // + + -
    envs_.push_back({{var_x_, -4.7}, {var_y_, -3}, {var_z_, 3.4}});   // - - +
    envs_.push_back({{var_x_, 3.1}, {var_y_, -3}, {var_z_, -2.5}});   // + - -
    envs_.push_back({{var_x_, -2.8}, {var_y_, 2}, {var_z_, -2.6}});   // _ + -
    envs_.push_back({{var_x_, -2.2}, {var_y_, -4}, {var_z_, -2.3}});  // - - -
  }

  // Check if both e and e.Expand() are evaluated to the close-enough (<eps)
  // values under all symbolic environments in envs_.
  bool CheckExpandPreserveEvaluation(const Expression& e, const double eps) {
    return all_of(envs_.begin(), envs_.end(), [&](const Environment& env) {
      return std::fabs(e.Evaluate(env) - e.Expand().Evaluate(env)) < eps;
    });
  }

  // Checks if e == e.Expand().
  bool CheckAlreadyExpanded(const Expression& e) {
    return e.EqualTo(e.Expand());
  }

  // Checks if e.Expand() == e.Expand().Expand().
  bool CheckExpandIsFixpoint(const Expression& e) {
    return e.Expand().EqualTo(e.Expand().Expand());
  }
};

TEST_F(SymbolicExpansionTest, ExpressionAlreadyExpandedPolynomial) {
  // The following are all already expanded.
  EXPECT_TRUE(CheckAlreadyExpanded(0));
  EXPECT_TRUE(CheckAlreadyExpanded(1));
  EXPECT_TRUE(CheckAlreadyExpanded(-1));
  EXPECT_TRUE(CheckAlreadyExpanded(42));
  EXPECT_TRUE(CheckAlreadyExpanded(-5));
  EXPECT_TRUE(CheckAlreadyExpanded(x_));
  EXPECT_TRUE(CheckAlreadyExpanded(-x_));
  EXPECT_TRUE(CheckAlreadyExpanded(3 * x_));
  EXPECT_TRUE(CheckAlreadyExpanded(-2 * x_));
  EXPECT_TRUE(CheckAlreadyExpanded(3 * x_ * y_));               // 3xy
  EXPECT_TRUE(CheckAlreadyExpanded(3 * pow(x_, 2) * y_));       // 3x^2y
  EXPECT_TRUE(CheckAlreadyExpanded(3 / 10 * pow(x_, 2) * y_));  // 3/10*x^2y
  EXPECT_TRUE(CheckAlreadyExpanded(-7 + x_ + y_));              // -7 + x + y
  EXPECT_TRUE(CheckAlreadyExpanded(1 + 3 * x_ - 4 * y_));       // 1 + 3x -4y
}

TEST_F(SymbolicExpansionTest, ExpressionAlreadyExpandedPow) {
  // The following are all already expanded.
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_, y_)));            // x^y
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_, -1)));            // x^(-1)
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_, -1)));            // x^(-1)
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_ + y_, -1)));       // (x + y)^(-1)
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_ + y_, 0.5)));      // (x + y)^(0.5)
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_ + y_, 2.5)));      // (x + y)^(2.5)
  EXPECT_TRUE(CheckAlreadyExpanded(pow(x_ + y_, x_ - y_)));  // (x + y)^(x - y)
}

TEST_F(SymbolicExpansionTest, ExpressionExpansion) {
  // test_exprs includes pairs of expression `e` and its expected expansion
  // `expected`. For each pair (e, expected), we check the following:
  //     1. e.Expand() is structurally equal to expected.
  //     2. Evaluate e and e.Expand() under multiple environments to check the
  //        correctness of expansions.
  //     3. A expansion is a fixpoint of Expand() function. That is, a expanded
  //        expression shouldn't be expanded further.

  vector<pair<Expression, Expression>> test_exprs;

  // (2xy²)² = 4x²y⁴
  test_exprs.emplace_back(pow(2 * x_ * y_ * y_, 2),
                          4 * pow(x_, 2) * pow(y_, 4));

  //   5 * (3 + 2y) + 30 * (7 + x_)
  // = 15 + 10y + 210 + 30x
  // = 225 + 30x + 10y
  test_exprs.emplace_back(5 * (3 + 2 * y_) + 30 * (7 + x_),
                          225 + 30 * x_ + 10 * y_);

  // (x + 3y) * (2x + 5y) = 2x^2 + 11xy + 15y^2
  test_exprs.emplace_back((x_ + 3 * y_) * (2 * x_ + 5 * y_),
                          2 * pow(x_, 2) + 11 * x_ * y_ + 15 * pow(y_, 2));

  //   (7 + x) * (5 + y) * (6 + z)
  // = (35 + 5x + 7y + xy) * (6 + z)
  // = (210 + 30x + 42y + 6xy) + (35z + 5xz + 7yz + xyz)
  test_exprs.emplace_back((7 + x_) * (5 + y_) * (6 + z_),
                          210 + 30 * x_ + 42 * y_ + 6 * x_ * y_ + 35 * z_ +
                              5 * x_ * z_ + 7 * y_ * z_ + x_ * y_ * z_);

  //   (x + 3y) * (2x + 5y) * (x + 3y)
  // = (2x^2 + 11xy + 15y^2) * (x + 3y)
  // = 2x^3 + 11x^2y + 15xy^2
  //        +  6x^2y + 33xy^2 + 45y^3
  // = 2x^3 + 17x^2y + 48xy^2 + 45y^3
  test_exprs.emplace_back((x_ + 3 * y_) * (2 * x_ + 5 * y_) * (x_ + 3 * y_),
                          2 * pow(x_, 3) + 17 * pow(x_, 2) * y_ +
                              48 * x_ * pow(y_, 2) + 45 * pow(y_, 3));

  //   pow((x + y)^2 + 1, (x - y)^2)
  // = pow(x^2 + 2xy + y^2 + 1, x^2 -2xy + y^2)
  // Expand the base and exponent of pow.
  test_exprs.emplace_back(pow(pow(x_ + y_, 2) + 1, pow(x_ - y_, 2)),
                          pow(pow(x_, 2) + 2 * x_ * y_ + pow(y_, 2) + 1,
                              pow(x_, 2) - 2 * x_ * y_ + pow(y_, 2)));

  //   (x + y + 1)^3
  // = x^3 + 3x^2y +
  //   3x^2 + 3xy^2 + 6xy + 3x +
  //   y^3 + 3y^2 + 3y + 1
  test_exprs.emplace_back(pow(x_ + y_ + 1, 3),
                          pow(x_, 3) + 3 * pow(x_, 2) * y_ + 3 * pow(x_, 2) +
                              3 * x_ * pow(y_, 2) + 6 * x_ * y_ + 3 * x_ +
                              pow(y_, 3) + 3 * pow(y_, 2) + 3 * y_ + 1);

  // (x + y + 1)^4
  // = 1 + 4x + 4y + 12xy + 12xy^2 +
  //       4xy^3 + 12x^2y +
  //       6x^2y^2 + 4x^3y +
  //       6x^2 + 4x^3 + x^4 +
  //       6y^2 + 4y^3 + y^4
  test_exprs.emplace_back(
      pow(x_ + y_ + 1, 4),
      1 + 4 * x_ + 4 * y_ + 12 * x_ * y_ + 12 * x_ * pow(y_, 2) +
          4 * x_ * pow(y_, 3) + 12 * pow(x_, 2) * y_ +
          6 * pow(x_, 2) * pow(y_, 2) + 4 * pow(x_, 3) * y_ + 6 * pow(x_, 2) +
          4 * pow(x_, 3) + pow(x_, 4) + 6 * pow(y_, 2) + 4 * pow(y_, 3) +
          pow(y_, 4));

  for (const pair<Expression, Expression>& p : test_exprs) {
    const Expression& e{p.first};
    const Expression expanded{e.Expand()};
    const Expression& expected{p.second};
    EXPECT_PRED2(ExprEqual, expanded, expected);
    EXPECT_TRUE(expanded.is_expanded());
    EXPECT_TRUE(CheckExpandPreserveEvaluation(e, 1e-8));
    EXPECT_TRUE(CheckExpandIsFixpoint(e));
  }
}

TEST_F(SymbolicExpansionTest, MathFunctions) {
  // For a math function f(x) and an expression e, we need to have the following
  // property:
  //
  //    f(e).Expand() == f(e.Expand())
  //
  // where '==' is structural equality (Expression::EqualTo).
  using F = function<Expression(const Expression&)>;
  vector<F> contexts;
  contexts.push_back([](const Expression& x) { return log(x); });
  contexts.push_back([](const Expression& x) { return abs(x); });
  contexts.push_back([](const Expression& x) { return exp(x); });
  contexts.push_back([](const Expression& x) { return sqrt(x); });
  contexts.push_back([](const Expression& x) { return sin(x); });
  contexts.push_back([](const Expression& x) { return cos(x); });
  contexts.push_back([](const Expression& x) { return tan(x); });
  contexts.push_back([](const Expression& x) { return asin(x); });
  contexts.push_back([](const Expression& x) { return acos(x); });
  contexts.push_back([](const Expression& x) { return atan(x); });
  contexts.push_back([](const Expression& x) { return sinh(x); });
  contexts.push_back([](const Expression& x) { return cosh(x); });
  contexts.push_back([](const Expression& x) { return tanh(x); });
  contexts.push_back([&](const Expression& x) { return min(x, y_); });
  contexts.push_back([&](const Expression& x) { return min(y_, x); });
  contexts.push_back([&](const Expression& x) { return max(x, z_); });
  contexts.push_back([&](const Expression& x) { return max(z_, x); });
  contexts.push_back([](const Expression& x) { return ceil(x); });
  contexts.push_back([](const Expression& x) { return floor(x); });
  contexts.push_back([&](const Expression& x) { return atan2(x, y_); });
  contexts.push_back([&](const Expression& x) { return atan2(y_, x); });

  vector<Expression> expressions;
  expressions.push_back(5 * (3 + 2 * y_) + 30 * (7 + x_));
  expressions.push_back((x_ + 3 * y_) * (2 * x_ + 5 * y_));
  expressions.push_back((7 + x_) * (5 + y_) * (6 + z_));
  expressions.push_back((x_ + 3 * y_) * (2 * x_ + 5 * y_) * (x_ + 3 * y_));
  expressions.push_back(pow(pow(x_ + y_, 2) + 1, pow(x_ - y_, 2)));
  expressions.push_back(pow(x_ + y_ + 1, 3));
  expressions.push_back(pow(x_ + y_ + 1, 4));

  for (const F& f : contexts) {
    for (const Expression& e : expressions) {
      const Expression e1{f(e).Expand()};
      const Expression e2{f(e.Expand())};

      EXPECT_PRED2(ExprEqual, e1, e2);
      EXPECT_TRUE(e1.is_expanded());
      EXPECT_TRUE(e2.is_expanded());
      EXPECT_TRUE(CheckAlreadyExpanded(e1));
    }
  }
}

TEST_F(SymbolicExpansionTest, NaN) {
  // NaN is considered as not expanded so that ExpressionNaN::Expand() is called
  // and throws an exception.
  EXPECT_FALSE(Expression::NaN().is_expanded());
  // NaN should be detected during expansion and throw runtime_error.
  Expression dummy;
  EXPECT_THROW(dummy = Expression::NaN().Expand(), runtime_error);
}

TEST_F(SymbolicExpansionTest, IfThenElse) {
  const Expression e{if_then_else(x_ > y_, pow(x_ + y_, 2), pow(x_ - y_, 2))};
  Expression dummy;
  EXPECT_THROW(dummy = e.Expand(), runtime_error);
  // An if-then-else expression is considered as not expanded so that
  // ExpressionIfThenElse::Expand() is called and throws an exception.
  EXPECT_FALSE(e.is_expanded());
}

TEST_F(SymbolicExpansionTest, UninterpretedFunction) {
  const Expression uf1{uninterpreted_function("uf1", {})};
  EXPECT_PRED2(ExprEqual, uf1, uf1.Expand());
  EXPECT_TRUE(uf1.Expand().is_expanded());

  const Expression e1{3 * (x_ + y_)};
  const Expression e2{pow(x_ + y_, 2)};
  const Expression uf2{uninterpreted_function("uf2", {e1, e2})};
  EXPECT_PRED2(ExprNotEqual, uf2, uf2.Expand());
  EXPECT_TRUE(uf2.Expand().is_expanded());
  const Expression uf2_expand_expected{
      uninterpreted_function("uf2", {e1.Expand(), e2.Expand()})};
  EXPECT_PRED2(ExprEqual, uf2.Expand(), uf2_expand_expected);
}

TEST_F(SymbolicExpansionTest, DivideByConstant) {
  // (x) / 2 => 0.5 * x
  EXPECT_PRED2(ExprEqual, (x_ / 2).Expand(), 0.5 * x_);

  // 3 / 2 => 3 / 2  (no simplification)
  EXPECT_PRED2(ExprEqual, (Expression(3.0) / 2).Expand(), 3.0 / 2);

  // pow(x, y) / 2 => 0.5 * pow(x, y)
  EXPECT_PRED2(ExprEqual, (pow(x_, y_) / 2).Expand(), 0.5 * pow(x_, y_));

  // (2x) / 2 => x
  EXPECT_PRED2(ExprEqual, ((2 * x_) / 2).Expand(), x_);

  // (10x / 5) / 2 => x
  EXPECT_PRED2(ExprEqual, (10 * x_ / 5 / 2).Expand(), x_);

  // (10x²y³z⁴) / -5 => -2x²y³z⁴
  EXPECT_PRED2(ExprEqual,
               (10 * pow(x_, 2) * pow(y_, 3) * pow(z_, 4) / -5).Expand(),
               -2 * pow(x_, 2) * pow(y_, 3) * pow(z_, 4));

  // (36xy / 4 / -3) => -3xy
  EXPECT_PRED2(ExprEqual, (36 * x_ * y_ / 4 / -3).Expand(), -3 * x_ * y_);

  std::cerr << (x_ / 2).is_polynomial() << std::endl;

  // (2x + 4xy + 6) / 2 => x + 2xy + 3
  EXPECT_PRED2(ExprEqual, ((2 * x_ + 4 * x_ * y_ + 6) / 2).Expand(),
               x_ + 2 * x_ * y_ + 3);

  // (4x / 3) * (6y / 2) => 4xy
  EXPECT_PRED2(ExprEqual, ((4 * x_ / 3) * (6 * y_ / 2)).Expand(), 4 * x_ * y_);

  // (6xy / z / 3) => 2xy / z
  EXPECT_PRED2(ExprEqual, (6 * x_ * y_ / z_ / 3).Expand(), 2 * x_ * y_ / z_);

  // (36xy / x / -3) => -12xy / x
  // Note that we do not cancel x out since it can be zero.
  EXPECT_PRED2(ExprEqual, (36 * x_ * y_ / x_ / -3).Expand(),
               -12 * x_ * y_ / x_);
}

TEST_F(SymbolicExpansionTest, RepeatedExpandShouldBeNoop) {
  const Expression e{(x_ + y_) * (x_ + y_)};

  // New ExpressionCells are created here.
  const Expression e_expanded{e.Expand()};
  {
    LimitMalloc guard;
    // e_expanded is already expanded, so the following line should not create a
    // new cell and no memory allocation should occur. We use LimitMalloc to
    // check this claim.
    const Expression e_expanded_expanded = e_expanded.Expand();
    EXPECT_PRED2(ExprEqual, e_expanded, e_expanded_expanded);
  }
}

TEST_F(SymbolicExpansionTest, ExpandMultiplicationsWithDivisions) {
  const Expression e1{((x_ + 1) / y_) * (x_ + 3)};
  const Expression e2{(x_ + 3) * ((x_ + 1) / z_)};
  const Expression e3{((x_ + 1) / (y_ + 6)) * ((x_ + 3) / (z_ + 7))};
  const Expression e4{(x_ + y_ / ((z_ + x_) * (y_ + x_))) * (x_ - y_ / z_) *
                      (x_ * y_ / z_)};

  EXPECT_TRUE(CheckExpandIsFixpoint(e1));
  EXPECT_TRUE(CheckExpandIsFixpoint(e2));
  EXPECT_TRUE(CheckExpandIsFixpoint(e3));
  EXPECT_TRUE(CheckExpandIsFixpoint(e4));

  EXPECT_TRUE(CheckExpandPreserveEvaluation(e1, 1e-8));
  EXPECT_TRUE(CheckExpandPreserveEvaluation(e2, 1e-8));
  EXPECT_TRUE(CheckExpandPreserveEvaluation(e3, 1e-8));
  EXPECT_TRUE(CheckExpandPreserveEvaluation(e4, 1e-8));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
