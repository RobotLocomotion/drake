#include "drake/common/symbolic_expression.h"

#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/symbolic_formula.h"
#include "drake/common/test/symbolic_test_util.h"
#include "drake/common/variable.h"

using std::function;
using std::is_same;
using std::pair;
using std::result_of;
using std::exception;
using std::vector;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

// Checks if 'Expression::Substitute(const Variable&, const Expression&)' is
// a homomorphism. That is, we check if the following holds:
//
//     f(v).Substitute(v, e) = f(e)
//
void CheckHomomorphism(const function<Expression(const Expression&)>& f,
                       const Variable& var, const Expression& expr) {
  try {
    const Expression apply_subst{f(Expression{var}).Substitute(var, expr)};
    const Expression subst_apply{f(expr)};
    EXPECT_PRED2(ExprEqual, apply_subst, subst_apply);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(expr));
  }
}

// Checks if 'Expression::Substitute(const Substitution&)' is a homomorphism.
// That is, we check if the following holds:
//
//     f({x_1, ..., x_n}).Substitute(s) = f({e_1, ..., e_n})
//
// where we have x_i.Substitute(s) = e_i by a given substitution s.
void CheckHomomorphism(const function<Expression(const vector<Expression>&)>& f,
                       const Substitution& s) {
  vector<Expression> args1;  // {x_1, ..., x_n}
  vector<Expression> args2;  // {e_1, ..., e_n}
  for (const pair<Variable, Expression>& p : s) {
    args1.emplace_back(p.first);
    args2.push_back(p.second);
  }

  try {
    const Expression apply_subst{f(args1).Substitute(s)};
    const Expression subst_apply{f(args2)};
    EXPECT_PRED2(ExprEqual, apply_subst, subst_apply);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(args2));
  }
}

class SymbolicSubstitutionTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
};

TEST_F(SymbolicSubstitutionTest, CheckHomomorphismExpressionVarExpr) {
  using F = function<Expression(const Expression&)>;

  vector<F> fns;
  fns.push_back([](const Expression& x) { return 3.0; });
  fns.push_back([](const Expression& x) { return x; });
  fns.push_back([](const Expression& x) { return -x; });
  fns.push_back([&](const Expression& x) { return x + y_; });
  fns.push_back([&](const Expression& x) { return y_ + x; });
  fns.push_back([&](const Expression& x) { return x - y_; });
  fns.push_back([&](const Expression& x) { return y_ - x; });
  fns.push_back([&](const Expression& x) { return x * z_; });
  fns.push_back([&](const Expression& x) { return z_ * x; });
  fns.push_back([&](const Expression& x) { return x / y_; });
  fns.push_back([&](const Expression& x) { return y_ / x; });
  fns.push_back([](const Expression& x) { return log(x); });
  fns.push_back([](const Expression& x) { return abs(x); });
  fns.push_back([](const Expression& x) { return exp(x); });
  fns.push_back([](const Expression& x) { return sqrt(x); });
  fns.push_back([&](const Expression& x) { return pow(x, y_); });
  fns.push_back([&](const Expression& x) { return pow(y_, x); });
  fns.push_back([](const Expression& x) { return sin(x); });
  fns.push_back([](const Expression& x) { return cos(x); });
  fns.push_back([](const Expression& x) { return tan(x); });
  fns.push_back([](const Expression& x) { return asin(x); });
  fns.push_back([](const Expression& x) { return acos(x); });
  fns.push_back([](const Expression& x) { return atan(x); });
  fns.push_back([&](const Expression& x) { return atan2(x, y_); });
  fns.push_back([&](const Expression& x) { return atan2(y_, x); });
  fns.push_back([](const Expression& x) { return sinh(x); });
  fns.push_back([](const Expression& x) { return cosh(x); });
  fns.push_back([](const Expression& x) { return tanh(x); });
  fns.push_back([&](const Expression& x) { return min(x, y_); });
  fns.push_back([&](const Expression& x) { return min(y_, x); });
  fns.push_back([&](const Expression& x) { return max(x, z_); });
  fns.push_back([&](const Expression& x) { return max(z_, x); });

  // TODO(soonho): enable the following tests when Formula::Substitute is
  // implemented.
  // fns.push_back([&](const Expression& x) {
  //   return if_then_else(x > y_ && x > z_, x * y_, x / z_);
  // });
  // fns.push_back([&](const Expression& x) {
  //   return if_then_else(x > y_ || z_ > x, x * y_, x / z_);
  // });

  vector<pair<Variable, Expression>> substs;
  substs.emplace_back(var_x_, x_);
  substs.emplace_back(var_x_, 1.0);
  substs.emplace_back(var_x_, -1.0);
  substs.emplace_back(var_x_, 20.0);
  substs.emplace_back(var_x_, -30.0);
  substs.emplace_back(var_x_, x_ + y_);
  substs.emplace_back(var_x_, y_ + z_);
  substs.emplace_back(var_x_, x_ - y_);
  substs.emplace_back(var_x_, y_ - z_);
  substs.emplace_back(var_x_, x_ * y_);
  substs.emplace_back(var_x_, y_ * z_);
  substs.emplace_back(var_x_, x_ / y_);
  substs.emplace_back(var_x_, y_ / z_);
  substs.emplace_back(var_x_, x_ - y_);
  substs.emplace_back(var_x_, y_ - z_);

  for (const F& f : fns) {
    for (const pair<Variable, Expression>& s : substs) {
      const Variable& var{s.first};
      const Expression& expr{s.second};
      CheckHomomorphism(f, var, expr);
    }
  }
}

TEST_F(SymbolicSubstitutionTest, CheckHomomorphismExpressionSubstitution) {
  using F = function<Expression(const vector<Expression>&)>;

  vector<F> fns;
  fns.push_back([](const vector<Expression>& v) { return 3.0; });
  fns.push_back([](const vector<Expression>& v) { return v[0]; });
  fns.push_back([](const vector<Expression>& v) { return -v[0]; });
  fns.push_back([](const vector<Expression>& v) { return v[0] + v[1]; });
  fns.push_back([](const vector<Expression>& v) { return v[1] - v[2]; });
  fns.push_back([](const vector<Expression>& v) { return v[0] * v[2]; });
  fns.push_back([](const vector<Expression>& v) { return v[0] / v[1]; });
  fns.push_back([](const vector<Expression>& v) { return log(v[0]); });
  fns.push_back([](const vector<Expression>& v) { return abs(v[1]); });
  fns.push_back([](const vector<Expression>& v) { return exp(v[2]); });
  fns.push_back([](const vector<Expression>& v) { return sqrt(v[0]); });
  fns.push_back([](const vector<Expression>& v) { return pow(v[0], v[1]); });
  fns.push_back([](const vector<Expression>& v) { return sin(v[1]); });
  fns.push_back([](const vector<Expression>& v) { return cos(v[2]); });
  fns.push_back([](const vector<Expression>& v) { return tan(v[0]); });
  fns.push_back([](const vector<Expression>& v) { return asin(v[0]); });
  fns.push_back([](const vector<Expression>& v) { return acos(v[1]); });
  fns.push_back([](const vector<Expression>& v) { return atan(v[2]); });
  fns.push_back([](const vector<Expression>& v) { return atan2(v[0], v[1]); });
  fns.push_back([](const vector<Expression>& v) { return sinh(v[1]); });
  fns.push_back([](const vector<Expression>& v) { return cosh(v[0]); });
  fns.push_back([](const vector<Expression>& v) { return tanh(v[2]); });
  fns.push_back([](const vector<Expression>& v) { return min(v[0], v[1]); });
  fns.push_back([](const vector<Expression>& v) { return max(v[1], v[2]); });
  fns.push_back([&](const vector<Expression>& v) {
    return fns[9](v) * fns[17](v) / fns[5](v) - fns[19](v);
  });
  fns.push_back([&](const vector<Expression>& v) {
    return fns[6](v) * fns[20](v) / fns[2](v) + fns[12](v);
  });

  // TODO(soonho): enable the following test when Formula::Substitute is
  // implemented.
  // fns.push_back([](const vector<Expression>& v) {
  //   return if_then_else(v[0] > v[1], v[1] * v[2], v[0] - v[2]);
  // });

  vector<Substitution> substs;
  substs.push_back({{var_x_, 1.0}, {var_y_, 1.0}, {var_z_, 2.0}});
  substs.push_back({{var_x_, -2.0}, {var_y_, 1.0}, {var_z_, z_}});
  substs.push_back({{var_x_, 0.0}, {var_y_, 0.0}, {var_z_, 5.0}});
  substs.push_back({{var_x_, -10.0}, {var_y_, 10.0}, {var_z_, 0.0}});
  substs.push_back({{var_x_, y_}, {var_y_, z_}, {var_z_, x_}});
  substs.push_back({{var_x_, x_ + y_}, {var_y_, y_ + z_}, {var_z_, z_ + x_}});
  substs.push_back({{var_x_, pow(x_, y_)},
                    {var_y_, sin(y_) + cos(z_)},
                    {var_z_, sqrt(x_ * y_ * z_)}});
  substs.push_back({{var_x_, pow(x_, y_)},
                    {var_y_, sin(y_) + cos(z_)},
                    {var_z_, log(pow(x_, y_) * z_)}});

  for (const F& f : fns) {
    for (const Substitution& s : substs) {
      CheckHomomorphism(f, s);
    }
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
