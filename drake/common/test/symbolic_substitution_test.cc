#include <functional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

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
using test::FormulaEqual;

// Checks if 'Expression::Substitute(const Variable&, const Expression&)' is
// a homomorphism. That is, we check if the following holds:
//
//     f(v).Substitute(v, e) = f(e)
//
void CheckHomomorphism(const function<Expression(const Expression&)>& f,
                       const Variable& var, const Expression& expr) {
  Expression apply_subst{0.0};
  try {
    apply_subst = f(Expression{var}).Substitute(var, expr);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(expr));
    return;
  }
  // Otherwise, we check if we have apply_subst = subst_apply.
  const Expression subst_apply{f(expr)};
  EXPECT_PRED2(ExprEqual, apply_subst, subst_apply);
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

  Expression apply_subst{0.0};
  try {
    apply_subst = f(args1).Substitute(s);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(args2));
    return;
  }
  // Otherwise, we check if we have apply_subst = subst_apply.
  const Expression subst_apply{f(args2)};
  EXPECT_PRED2(ExprEqual, apply_subst, subst_apply);
}

// Checks if 'Formula::Substitute(const Variable&, const Expression&)' is
// a homomorphism. That is, we check if the following holds:
//
//     f(v).Substitute(v, e) = f(e)
//
// Note that the above assertion holds only if f is a quantifier-free
// formula. We have a separate tests which covers the quantified case.
void CheckHomomorphism(const function<Formula(const Expression&)>& f,
                       const Variable& var, const Expression& expr) {
  Formula apply_subst{Formula::True()};
  try {
    apply_subst = f(Expression{var}).Substitute(var, expr);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(expr));
    return;
  }
  // Otherwise, we check if we have apply_subst = subst_apply.
  const Formula subst_apply{f(expr)};
  EXPECT_PRED2(FormulaEqual, apply_subst, subst_apply);
}

// Checks if 'Formula::Substitute(const Substitution&)' is a homomorphism.
// That is, we check if the following holds:
//
//     f({x_1, ..., x_n}).Substitute(s) = f({e_1, ..., e_n})
//
// where we have x_i.Substitute(s) = e_i by a given substitution s.
//
// Note that the above assertion holds only if f is a quantifier-free
// formula. We have a separate tests which covers the quantified case.
void CheckHomomorphism(const function<Formula(const vector<Expression>&)>& f,
                       const Substitution& s) {
  vector<Expression> args1;  // {x_1, ..., x_n}
  vector<Expression> args2;  // {e_1, ..., e_n}
  for (const pair<Variable, Expression>& p : s) {
    args1.emplace_back(p.first);
    args2.push_back(p.second);
  }

  Formula apply_subst{Formula::True()};
  try {
    apply_subst = f(args1).Substitute(s);
  } catch (const exception&) {
    // If apply_subst throws an exception, then subst_apply should
    // throws an exception as well.
    EXPECT_ANY_THROW(f(args2));
    return;
  }
  // Otherwise, we check if we have apply_subst = subst_apply.
  const Formula subst_apply{f(args2)};
  EXPECT_PRED2(FormulaEqual, apply_subst, subst_apply);
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
  fns.push_back([](const Expression& x) { return 2 * x; });
  fns.push_back([](const Expression& x) { return -x; });
  fns.push_back([](const Expression& x) { return -3 * x; });
  fns.push_back([&](const Expression& x) { return 5.0 + x + y_; });
  fns.push_back([&](const Expression& x) { return 5.0 + y_ + x; });
  fns.push_back([&](const Expression& x) { return 7.0 - x - y_; });
  fns.push_back([&](const Expression& x) { return 7.0 - y_ - x; });
  fns.push_back([&](const Expression& x) { return 3.0 * x * z_; });
  fns.push_back([&](const Expression& x) { return 3.0 * z_ * x; });
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
  fns.push_back([&](const Expression& x) {
    return if_then_else(x > y_ && x > z_, x * y_, x / z_);
  });
  fns.push_back([&](const Expression& x) {
    return if_then_else(x > y_ || z_ > x, x * y_, x / z_);
  });

  vector<pair<Variable, Expression>> substs;
  substs.emplace_back(var_x_, x_);
  substs.emplace_back(var_x_, 1.0);
  substs.emplace_back(var_x_, -1.0);
  substs.emplace_back(var_x_, 20.0);
  substs.emplace_back(var_x_, -30.0);
  substs.emplace_back(var_x_, 7.0 + x_ + y_);
  substs.emplace_back(var_x_, -3.0 + y_ + z_);
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
  fns.push_back([](const vector<Expression>& v) { return 2 * v[0]; });
  fns.push_back([](const vector<Expression>& v) { return -v[0]; });
  fns.push_back([](const vector<Expression>& v) { return -3 * v[0]; });
  fns.push_back([](const vector<Expression>& v) { return 3.0 + v[0] + v[1]; });
  fns.push_back([](const vector<Expression>& v) { return -3.0 + v[1] - v[2]; });
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
  fns.push_back([](const vector<Expression>& v) {
    return if_then_else(v[0] > v[1], v[1] * v[2], v[0] - v[2]);
  });

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

TEST_F(SymbolicSubstitutionTest, CheckHomomorphismFormulaVarExpr) {
  using F = function<Formula(const Expression&)>;

  vector<F> fns;
  fns.push_back([](const Expression& x) { return Formula::True(); });
  fns.push_back([](const Expression& x) { return Formula::False(); });
  fns.push_back([&](const Expression& x) { return (x + y_) == (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) == (x * z_); });
  fns.push_back([&](const Expression& x) { return (x + y_) != (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) != (x * z_); });
  fns.push_back([&](const Expression& x) { return (x + y_) > (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) > (x * z_); });
  fns.push_back([&](const Expression& x) { return (x + y_) >= (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) >= (x * z_); });
  fns.push_back([&](const Expression& x) { return (x + y_) < (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) < (x * z_); });
  fns.push_back([&](const Expression& x) { return (x + y_) <= (y_ * z_); });
  fns.push_back([&](const Expression& x) { return (y_ + z_) <= (x * z_); });
  fns.push_back([&](const Expression& x) { return fns[5](x) && fns[7](x); });
  fns.push_back([&](const Expression& x) { return fns[2](x) || fns[6](x); });
  fns.push_back([&](const Expression& x) { return !fns[14](x); });
  fns.push_back([&](const Expression& x) { return !fns[15](x); });

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

TEST_F(SymbolicSubstitutionTest, CheckHomomorphismFormulaSubstitution) {
  using F = function<Formula(const vector<Expression>&)>;

  vector<F> fns;
  fns.push_back([](const vector<Expression>& v) { return Formula::True(); });
  fns.push_back([](const vector<Expression>& v) { return Formula::False(); });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) == (v[1] * v[2]);
  });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) != (v[1] * v[2]);
  });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) > (v[1] * v[2]);
  });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) >= (v[1] * v[2]);
  });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) < (v[1] * v[2]);
  });
  fns.push_back([](const vector<Expression>& v) {
    return (v[0] + v[1]) <= (v[1] * v[2]);
  });
  fns.push_back(
      [&](const vector<Expression>& v) { return fns[5](v) && fns[7](v); });
  fns.push_back(
      [&](const vector<Expression>& v) { return fns[2](v) || fns[4](v); });
  fns.push_back([&](const vector<Expression>& v) { return !fns[8](v); });
  fns.push_back([&](const vector<Expression>& v) { return !fns[9](v); });

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

class ForallFormulaSubstitutionTest : public SymbolicSubstitutionTest {
 protected:
  const Expression e_{x_ + y_ + z_};
  const Formula f1_{x_ + y_ > z_};
  const Formula f2_{x_ * y_ < 5 * z_};
  const Formula f3_{x_ / y_ < 5 * z_};
  const Formula f4_{x_ - y_ < 5 * z_};
  const Formula f5_{e_ == 0.0};
  const Formula f6_{e_ != 0.0};
  const Formula f7_{e_ < 0.0};
  const Formula f8_{e_ <= 0.0};
  const Formula f9_{e_ > 0.0};
  const Formula f10_{e_ >= 0.0};
  const Formula f11_{f1_ && f2_};
  const Formula f12_{f1_ || f2_};
  const Formula f13_{!f11_};
  const Formula f14_{!f12_};
  const vector<Formula> formulas_{f1_, f2_, f3_,  f4_,  f5_,  f6_,  f7_,
                                  f8_, f9_, f10_, f11_, f12_, f13_, f14_};

  const Formula forall_x_1_{forall({var_x_}, f1_)};
  const Formula forall_x_2_{forall({var_x_}, f2_)};
  const Formula forall_x_3_{forall({var_x_}, f3_)};
  const Formula forall_x_4_{forall({var_x_}, f4_)};
  const Formula forall_x_5_{forall({var_x_}, f5_)};
  const Formula forall_x_6_{forall({var_x_}, f6_)};
  const Formula forall_x_7_{forall({var_x_}, f7_)};
  const Formula forall_x_8_{forall({var_x_}, f8_)};
  const Formula forall_x_9_{forall({var_x_}, f9_)};
  const Formula forall_x_10_{forall({var_x_}, f10_)};
  const Formula forall_x_11_{forall({var_x_}, f11_)};
  const Formula forall_x_12_{forall({var_x_}, f12_)};
  const Formula forall_x_13_{forall({var_x_}, f13_)};
  const Formula forall_x_14_{forall({var_x_}, f14_)};

  const vector<Formula> forall_formulas_{
      forall_x_1_,  forall_x_2_,  forall_x_3_,  forall_x_4_, forall_x_5_,
      forall_x_6_,  forall_x_7_,  forall_x_8_,  forall_x_9_, forall_x_10_,
      forall_x_11_, forall_x_12_, forall_x_13_, forall_x_14_};
};

TEST_F(ForallFormulaSubstitutionTest, VarExpr1) {
  vector<pair<Variable, Expression>> substs;
  substs.emplace_back(var_x_, 1.0);
  substs.emplace_back(var_x_, x_);
  substs.emplace_back(var_x_, 5 * x_);
  substs.emplace_back(var_x_, -x_);
  substs.emplace_back(var_x_, -2 * x_);
  substs.emplace_back(var_x_, y_);
  substs.emplace_back(var_x_, z_);

  for (const auto& f : forall_formulas_) {
    EXPECT_TRUE(is_forall(f));
    const Variables& vars{get_quantified_variables(f)};
    for (const auto& s : substs) {
      const Variable& var{s.first};
      const Expression& e{s.second};
      EXPECT_TRUE(vars.include(var));
      // var is a quantified variable, so Substitute doesn't change anything.
      EXPECT_PRED2(FormulaEqual, f, f.Substitute(var, e));
    }
  }
}

TEST_F(ForallFormulaSubstitutionTest, VarExpr2) {
  vector<pair<Variable, Expression>> substs;
  substs.emplace_back(var_y_, 1.0);
  substs.emplace_back(var_y_, y_);
  substs.emplace_back(var_y_, 5 * x_);
  substs.emplace_back(var_y_, -y_);
  substs.emplace_back(var_y_, -2 * x_);
  substs.emplace_back(var_y_, y_);
  substs.emplace_back(var_y_, z_);

  for (const auto& f : forall_formulas_) {
    EXPECT_TRUE(is_forall(f));
    const Variables& vars{get_quantified_variables(f)};
    const Formula& nested_f{get_quantified_formula(f)};
    for (const auto& s : substs) {
      const Variable& var{s.first};
      const Expression& e{s.second};
      EXPECT_FALSE(vars.include(var));

      // var is not a quantified variable, so the substitution goes inside of
      // the quantifier block. As a result, the following holds:
      //
      //     forall({v_1, ..., v_n}, f).subst(var, e)   -- (1)
      //   = forall({v_1, ..., v_n}, f.subst(var, e))   -- (2)
      //
      Formula f1{Formula::True()};
      try {
        f1 = {f.Substitute(var, e)};
      } catch (const exception&) {
        // If (1) throws an exception, then (2) should throws an exception as
        // well.
        EXPECT_ANY_THROW(forall(vars, nested_f.Substitute(var, e)));
        continue;
      }
      const Formula& f2{forall(vars, nested_f.Substitute(var, e))};
      EXPECT_PRED2(FormulaEqual, f1, f2);
    }
  }
}

TEST_F(ForallFormulaSubstitutionTest, VarExprSubstitution) {
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

  // In general, we have the following property:
  //
  //     forall(vars, f).subst(s)        -- (1)
  //   = forall(vars, f.subst(s∖vars))   -- (2)
  //
  // where vars = {v_1, ..., v_n} and s∖vars denotes a substitution which
  // includes the entries (v, e) ∈ s but v ∉ dom(s).
  //
  for (const auto& f : forall_formulas_) {
    EXPECT_TRUE(is_forall(f));
    const Variables& vars{get_quantified_variables(f)};
    const Formula& nested_f{get_quantified_formula(f)};

    for (const auto& s : substs) {
      Substitution s_minus_vars{s};
      for (const Variable& quantified_var : vars) {
        s_minus_vars.erase(quantified_var);
      }
      Formula f1{Formula::True()};
      try {
        f1 = {f.Substitute(s)};
      } catch (const exception&) {
        // If (1) throws an exception, then (2) should throws an exception as
        // well.
        EXPECT_ANY_THROW(forall(vars, nested_f.Substitute(s_minus_vars)));
        continue;
      }
      const Formula& f2{forall(vars, nested_f.Substitute(s_minus_vars))};
      EXPECT_PRED2(FormulaEqual, f1, f2);
    }
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
