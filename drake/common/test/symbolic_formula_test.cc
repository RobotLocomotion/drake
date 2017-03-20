#include "drake/common/symbolic_formula.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;

using test::all_of;
using test::any_of;
using test::ExprEqual;
using test::FormulaEqual;
using test::FormulaLess;
using test::FormulaNotEqual;
using test::FormulaNotLess;

// Checks if a given 'formulas' is ordered by Formula::Less.
static void CheckOrdering(const vector<Formula>& formulas) {
  for (size_t i{0}; i < formulas.size(); ++i) {
    for (size_t j{0}; j < formulas.size(); ++j) {
      if (i < j) {
        EXPECT_PRED2(FormulaLess, formulas[i], formulas[j])
            << "(Formulas[" << i << "] = " << formulas[i] << ")"
            << " is not less than "
            << "(Formulas[" << j << "] = " << formulas[j] << ")";
        EXPECT_PRED2(FormulaNotLess, formulas[j], formulas[i])
            << "(Formulas[" << j << "] = " << formulas[j] << ")"
            << " is less than "
            << "(Formulas[" << i << "] = " << formulas[i] << ")";
      } else if (i > j) {
        EXPECT_PRED2(FormulaLess, formulas[j], formulas[i])
            << "(Formulas[" << j << "] = " << formulas[j] << ")"
            << " is not less than "
            << "(Formulas[" << i << "] = " << formulas[i] << ")";
        EXPECT_PRED2(FormulaNotLess, formulas[i], formulas[j])
            << "(Formulas[" << i << "] = " << formulas[i] << ")"
            << " is less than "
            << "(Formulas[" << j << "] = " << formulas[j] << ")";
      } else {
        // i == j
        EXPECT_PRED2(FormulaNotLess, formulas[i], formulas[j])
            << "(Formulas[" << i << "] = " << formulas[i] << ")"
            << " is less than "
            << "(Formulas[" << j << "] = " << formulas[j] << ")";
        EXPECT_PRED2(FormulaNotLess, formulas[j], formulas[i])
            << "(Formulas[" << j << "] = " << formulas[j] << ")"
            << " is less than "
            << "(Formulas[" << i << "] = " << formulas[i] << ")";
      }
    }
  }
}

// Provides common variables that are used by the following tests.
class SymbolicFormulaTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Formula tt_{Formula::True()};
  const Formula ff_{Formula::False()};

  const Expression e1_{x_ + y_};
  const Expression e1_prime_{x_ + y_};
  const Expression e2_{x_ - y_};
  const Expression e3_{x_ + z_};

  const Formula f1_{x_ + y_ > 0};
  const Formula f2_{x_ * y_ < 5};
  const Formula f3_{x_ / y_ < 5};
  const Formula f4_{x_ - y_ < 5};
  const Formula f_eq_{e1_ == 0.0};
  const Formula f_neq_{e1_ != 0.0};
  const Formula f_lt_{e1_ < 0.0};
  const Formula f_lte_{e1_ <= 0.0};
  const Formula f_gt_{e1_ > 0.0};
  const Formula f_gte_{e1_ >= 0.0};
  const Formula f_and_{f1_ && f2_};
  const Formula f_or_{f1_ || f2_};
  const Formula not_f_or_{!f_or_};
  const Formula f_forall_{forall({var_x_, var_y_}, f_or_)};
  const Formula f_isnan_{isnan(Expression::NaN())};

  const Environment env1_{{var_x_, 1}, {var_y_, 1}};
  const Environment env2_{{var_x_, 3}, {var_y_, 4}};
  const Environment env3_{{var_x_, -2}, {var_y_, -5}};
  const Environment env4_{{var_x_, -1}, {var_y_, -1}};
};

TEST_F(SymbolicFormulaTest, LessKind) {
  // clang-format off
  CheckOrdering({
        Formula::False(),
        Formula::True(),
        x_ == y_,
        x_ != y_,
        x_> y_,
        x_ >= y_,
        x_ < y_,
        x_ <= y_,
        f1_ && f2_,
        f1_ || f2_,
        !f1_,
        f_forall_,
        f_isnan_});
  // clang-format on
}

TEST_F(SymbolicFormulaTest, LessTrueFalse) {
  CheckOrdering({Formula::False(), Formula::True()});
}

TEST_F(SymbolicFormulaTest, LessEq) {
  const Formula f1{x_ == y_};
  const Formula f2{x_ == z_};
  const Formula f3{y_ == z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessNeq) {
  const Formula f1{x_ != y_};
  const Formula f2{x_ != z_};
  const Formula f3{y_ != z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessGt) {
  const Formula f1{x_ > y_};
  const Formula f2{x_ > z_};
  const Formula f3{y_ > z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessGeq) {
  const Formula f1{x_ >= y_};
  const Formula f2{x_ >= z_};
  const Formula f3{y_ >= z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessLt) {
  const Formula f1{x_ < y_};
  const Formula f2{x_ < z_};
  const Formula f3{y_ < z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessLeq) {
  const Formula f1{x_ <= y_};
  const Formula f2{x_ <= z_};
  const Formula f3{y_ <= z_};
  CheckOrdering({f1, f2, f3});
}

TEST_F(SymbolicFormulaTest, LessAnd) {
  const Formula and1{f1_ && f2_ && f3_};
  const Formula and2{f1_ && f3_};
  const Formula and3{f2_ && f3_};
  CheckOrdering({and1, and2, and3});
}

TEST_F(SymbolicFormulaTest, LessOr) {
  const Formula or1{f1_ || f2_ || f3_};
  const Formula or2{f1_ || f3_};
  const Formula or3{f2_ || f3_};
  CheckOrdering({or1, or2, or3});
}

TEST_F(SymbolicFormulaTest, LessNot) {
  const Formula not1{!f1_};
  const Formula not2{!f2_};
  const Formula not3{!f3_};
  CheckOrdering({not1, not2, not3});
}

TEST_F(SymbolicFormulaTest, LessForall) {
  const Formula forall1{forall({var_x_, var_y_}, f1_)};
  const Formula forall2{forall({var_x_, var_y_, var_z_}, f1_)};
  const Formula forall3{forall({var_x_, var_y_, var_z_}, f2_)};
  CheckOrdering({forall1, forall2, forall3});
}

TEST_F(SymbolicFormulaTest, True) {
  EXPECT_PRED2(FormulaEqual, Formula::True(), Formula{tt_});
  EXPECT_TRUE(Formula::True().Evaluate());
  EXPECT_EQ(Formula::True().GetFreeVariables().size(), 0u);
  EXPECT_EQ(Formula::True().to_string(), "True");
  EXPECT_TRUE(is_true(Formula::True()));
}

TEST_F(SymbolicFormulaTest, False) {
  EXPECT_PRED2(FormulaEqual, Formula::False(), Formula{ff_});
  EXPECT_FALSE(Formula::False().Evaluate());
  EXPECT_EQ(Formula::False().GetFreeVariables().size(), 0u);
  EXPECT_EQ(Formula::False().to_string(), "False");
  EXPECT_TRUE(is_false(Formula::False()));
}

TEST_F(SymbolicFormulaTest, IsNaN) {
  // Things that aren't NaN are !isnan.
  const Expression zero{0};
  const Formula zero_is_nan{isnan(zero)};
  EXPECT_FALSE(zero_is_nan.Evaluate());

  // Things that _are_ NaN are exceptions, which is consistent with Expression
  // disallowing NaNs to be evaluated at runtime.
  const Expression nan{NAN};
  const Formula nan_is_nan{isnan(nan)};
  EXPECT_THROW(nan_is_nan.Evaluate(), std::runtime_error);

  // Buried NaNs are safe.
  const Formula ite_nan1{isnan(if_then_else(tt_, zero, nan))};
  EXPECT_FALSE(ite_nan1.Evaluate());

  // This case will be evaluated to NaN and we will have std::runtime_error.
  const Formula ite_nan2{isnan(if_then_else(ff_, zero, nan))};
  EXPECT_THROW(ite_nan2.Evaluate(), std::runtime_error);

  // Formula isnan(x / y) should throw a std::runtime_error when evaluated with
  // an environment mapping both of x and y to zero, because 0.0 / 0.0 = NaN.
  const Expression x_div_y{x_ / y_};
  const Environment env1{{var_x_, 0.0}, {var_y_, 0.0}};
  EXPECT_THROW(isnan(x_div_y).Evaluate(env1), std::runtime_error);

  // If the included expression `e` is not evaluated to NaN, `isnan(e)` should
  // return false.
  const Environment env2{{var_x_, 3.0}, {var_y_, 2.0}};
  EXPECT_FALSE(isnan(x_div_y).Evaluate(env2));
}

TEST_F(SymbolicFormulaTest, EqualTo1) {
  const Formula f_eq{x_ == y_};
  const Formula f_ne{x_ != y_};
  const Formula f_lt{x_ < y_};
  const Formula f_le{x_ <= y_};
  const Formula f_gt{x_ > y_};
  const Formula f_ge{x_ >= y_};

  EXPECT_PRED2(FormulaEqual, f_eq, f_eq);
  EXPECT_PRED2(FormulaEqual, f_eq, x_ == y_);
  EXPECT_PRED2(FormulaNotEqual, f_eq, x_ == z_);
  EXPECT_PRED2(FormulaNotEqual, f_eq, f_ne);
  EXPECT_PRED2(FormulaNotEqual, f_eq, f_lt);
  EXPECT_PRED2(FormulaNotEqual, f_eq, f_le);
  EXPECT_PRED2(FormulaNotEqual, f_eq, f_gt);
  EXPECT_PRED2(FormulaNotEqual, f_eq, f_ge);

  EXPECT_PRED2(FormulaNotEqual, f_ne, f_eq);
  EXPECT_PRED2(FormulaEqual, f_ne, f_ne);
  EXPECT_PRED2(FormulaEqual, f_ne, x_ != y_);
  EXPECT_PRED2(FormulaNotEqual, f_ne, x_ != z_);
  EXPECT_PRED2(FormulaNotEqual, f_ne, f_lt);
  EXPECT_PRED2(FormulaNotEqual, f_ne, f_le);
  EXPECT_PRED2(FormulaNotEqual, f_ne, f_gt);
  EXPECT_PRED2(FormulaNotEqual, f_ne, f_ge);

  EXPECT_PRED2(FormulaNotEqual, f_lt, f_eq);
  EXPECT_PRED2(FormulaNotEqual, f_lt, f_ne);
  EXPECT_PRED2(FormulaEqual, f_lt, f_lt);
  EXPECT_PRED2(FormulaEqual, f_lt, x_ < y_);
  EXPECT_PRED2(FormulaNotEqual, f_lt, x_ < z_);
  EXPECT_PRED2(FormulaNotEqual, f_lt, f_le);
  EXPECT_PRED2(FormulaNotEqual, f_lt, f_gt);
  EXPECT_PRED2(FormulaNotEqual, f_lt, f_ge);

  EXPECT_PRED2(FormulaNotEqual, f_le, f_eq);
  EXPECT_PRED2(FormulaNotEqual, f_le, f_ne);
  EXPECT_PRED2(FormulaNotEqual, f_le, f_lt);
  EXPECT_PRED2(FormulaEqual, f_le, f_le);
  EXPECT_PRED2(FormulaEqual, f_le, x_ <= y_);
  EXPECT_PRED2(FormulaNotEqual, f_le, x_ <= z_);
  EXPECT_PRED2(FormulaNotEqual, f_le, f_gt);
  EXPECT_PRED2(FormulaNotEqual, f_le, f_ge);

  EXPECT_PRED2(FormulaNotEqual, f_gt, f_eq);
  EXPECT_PRED2(FormulaNotEqual, f_gt, f_ne);
  EXPECT_PRED2(FormulaNotEqual, f_gt, f_lt);
  EXPECT_PRED2(FormulaNotEqual, f_gt, f_le);
  EXPECT_PRED2(FormulaEqual, f_gt, f_gt);
  EXPECT_PRED2(FormulaEqual, f_gt, x_ > y_);
  EXPECT_PRED2(FormulaNotEqual, f_gt, x_ > z_);
  EXPECT_PRED2(FormulaNotEqual, f_gt, f_ge);

  EXPECT_PRED2(FormulaNotEqual, f_ge, f_eq);
  EXPECT_PRED2(FormulaNotEqual, f_ge, f_ne);
  EXPECT_PRED2(FormulaNotEqual, f_ge, f_lt);
  EXPECT_PRED2(FormulaNotEqual, f_ge, f_le);
  EXPECT_PRED2(FormulaNotEqual, f_ge, f_gt);
  EXPECT_PRED2(FormulaEqual, f_ge, f_ge);
  EXPECT_PRED2(FormulaEqual, f_ge, x_ >= y_);
  EXPECT_PRED2(FormulaNotEqual, f_ge, x_ >= z_);
}

TEST_F(SymbolicFormulaTest, EqualTo2) {
  const Formula f1{x_ == y_};
  const Formula f2{y_ > z_};

  const Formula f_and{f1 && f2};
  const Formula f_or{f1 || f2};
  const Formula f_not{!f1};

  EXPECT_PRED2(FormulaEqual, f_and, f_and);
  EXPECT_PRED2(FormulaNotEqual, f_and, f_or);
  EXPECT_PRED2(FormulaNotEqual, f_and, f_not);

  EXPECT_PRED2(FormulaNotEqual, f_or, f_and);
  EXPECT_PRED2(FormulaEqual, f_or, f_or);
  EXPECT_PRED2(FormulaNotEqual, f_or, f_not);

  EXPECT_PRED2(FormulaNotEqual, f_not, f_and);
  EXPECT_PRED2(FormulaNotEqual, f_not, f_or);
  EXPECT_PRED2(FormulaEqual, f_not, f_not);
}

TEST_F(SymbolicFormulaTest, EqualTo3) {
  const Formula f_forall1{forall({var_x_, var_y_}, f_or_)};
  const Formula f_forall2{forall({var_x_, var_y_, var_z_}, f_or_)};
  const Formula f_forall3{forall({var_x_, var_y_}, f_and_)};
  const Formula f_forall4{forall({var_x_, var_y_, var_z_}, f_and_)};

  EXPECT_PRED2(FormulaEqual, f_forall1, f_forall1);
  EXPECT_PRED2(FormulaNotEqual, f_forall1, f_forall2);
  EXPECT_PRED2(FormulaNotEqual, f_forall1, f_forall3);
  EXPECT_PRED2(FormulaNotEqual, f_forall1, f_forall4);

  EXPECT_PRED2(FormulaNotEqual, f_forall2, f_forall1);
  EXPECT_PRED2(FormulaEqual, f_forall2, f_forall2);
  EXPECT_PRED2(FormulaNotEqual, f_forall2, f_forall3);
  EXPECT_PRED2(FormulaNotEqual, f_forall2, f_forall4);

  EXPECT_PRED2(FormulaNotEqual, f_forall3, f_forall1);
  EXPECT_PRED2(FormulaNotEqual, f_forall3, f_forall2);
  EXPECT_PRED2(FormulaEqual, f_forall3, f_forall3);
  EXPECT_PRED2(FormulaNotEqual, f_forall3, f_forall4);

  EXPECT_PRED2(FormulaNotEqual, f_forall4, f_forall1);
  EXPECT_PRED2(FormulaNotEqual, f_forall4, f_forall2);
  EXPECT_PRED2(FormulaNotEqual, f_forall4, f_forall3);
  EXPECT_PRED2(FormulaEqual, f_forall4, f_forall4);
}

TEST_F(SymbolicFormulaTest, Eq) {
  const Formula f1{e1_ == e1_};  // true
  EXPECT_PRED2(FormulaEqual, f1, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::False());
  const Formula f2{e1_ == e1_prime_};  // true
  EXPECT_PRED2(FormulaEqual, f2, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::False());
  const Formula f3{e1_ == e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::True());
  const Formula f4{e2_ == e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::True());
  const Formula f5{x_ == x_ + 5};  // false
  EXPECT_PRED2(FormulaEqual, f5, Formula::False());

  EXPECT_TRUE(all_of({f3, f4}, is_equal_to));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_equal_to));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) == (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) == (2 + 3));

  EXPECT_EQ((5 + x_ == 3 + y_).to_string(), "((5 + x) = (3 + y))");
}

TEST_F(SymbolicFormulaTest, Neq) {
  const Formula f1{e1_ != e1_};  // false
  EXPECT_PRED2(FormulaEqual, f1, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::True());
  const Formula f2{e1_ != e1_prime_};  // false
  EXPECT_PRED2(FormulaEqual, f2, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::True());
  const Formula f3{e1_ != e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::False());
  const Formula f4{e2_ != e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::False());
  const Formula f5{x_ != x_ + 5};  // true
  EXPECT_PRED2(FormulaEqual, f5, Formula::True());

  EXPECT_TRUE(all_of({f3, f4}, is_not_equal_to));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_not_equal_to));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) != (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) != (2 + 3));

  EXPECT_EQ((5 + x_ != 3 + y_).to_string(), "((5 + x) != (3 + y))");
}

TEST_F(SymbolicFormulaTest, Lt) {
  const Formula f1{e1_ < e1_};  // false
  EXPECT_PRED2(FormulaEqual, f1, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::True());
  const Formula f2{e1_ < e1_prime_};  // false
  EXPECT_PRED2(FormulaEqual, f2, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::True());
  const Formula f3{e1_ < e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::True());
  const Formula f4{e2_ < e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::True());
  const Formula f5{x_ < x_ + 5};  // true
  EXPECT_PRED2(FormulaEqual, f5, Formula::True());

  EXPECT_TRUE(all_of({f3, f4}, is_less_than));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_less_than));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) < (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) < (2 + 3));

  EXPECT_EQ((5 + x_ < 3 + y_).to_string(), "((5 + x) < (3 + y))");
}

TEST_F(SymbolicFormulaTest, Gt) {
  const Formula f1{e1_ > e1_};  // false
  EXPECT_PRED2(FormulaEqual, f1, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::True());
  const Formula f2{e1_ > e1_prime_};  // false
  EXPECT_PRED2(FormulaEqual, f2, Formula::False());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::True());
  const Formula f3{e1_ > e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::True());
  const Formula f4{e2_ > e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::True());
  const Formula f5{x_ > x_ + 5};  // false
  EXPECT_PRED2(FormulaEqual, f5, Formula::False());

  EXPECT_TRUE(all_of({f3, f4}, is_greater_than));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_greater_than));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) > (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) > (2 + 3));

  EXPECT_EQ((5 + x_ > 3 + y_).to_string(), "((5 + x) > (3 + y))");
}

TEST_F(SymbolicFormulaTest, Leq) {
  const Formula f1{e1_ <= e1_};  // true
  EXPECT_PRED2(FormulaEqual, f1, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::False());
  const Formula f2{e1_ <= e1_prime_};  // true
  EXPECT_PRED2(FormulaEqual, f2, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::False());
  const Formula f3{e1_ <= e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::True());
  const Formula f4{e2_ <= e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::True());
  const Formula f5{x_ <= x_ + 5};  // true
  EXPECT_PRED2(FormulaEqual, f5, Formula::True());

  EXPECT_TRUE(all_of({f3, f4}, is_less_than_or_equal_to));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_less_than_or_equal_to));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) <= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) <= (2 + 3));

  EXPECT_EQ((5 + x_ <= 3 + y_).to_string(), "((5 + x) <= (3 + y))");
}

TEST_F(SymbolicFormulaTest, Geq) {
  const Formula f1{e1_ >= e1_};  // true
  EXPECT_PRED2(FormulaEqual, f1, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f1, Formula::False());
  const Formula f2{e1_ >= e1_prime_};  // true
  EXPECT_PRED2(FormulaEqual, f2, Formula::True());
  EXPECT_PRED2(FormulaNotEqual, f2, Formula::False());
  const Formula f3{e1_ >= e3_};
  EXPECT_PRED2(FormulaNotEqual, f3, Formula::True());
  const Formula f4{e2_ >= e3_};
  EXPECT_PRED2(FormulaNotEqual, f4, Formula::True());
  const Formula f5{x_ >= x_ + 5};  // false
  EXPECT_PRED2(FormulaEqual, f5, Formula::False());

  EXPECT_TRUE(all_of({f3, f4}, is_greater_than_or_equal_to));
  EXPECT_FALSE(any_of({f1, f2, f5}, is_greater_than_or_equal_to));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) >= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) >= (2 + 3));

  EXPECT_EQ((5 + x_ >= 3 + y_).to_string(), "((5 + x) >= (3 + y))");
}

TEST_F(SymbolicFormulaTest, And1) {
  EXPECT_PRED2(FormulaEqual, tt_, tt_ && tt_);
  EXPECT_PRED2(FormulaEqual, ff_, ff_ && tt_);
  EXPECT_PRED2(FormulaEqual, ff_, tt_ && ff_);
  EXPECT_PRED2(FormulaEqual, ff_, ff_ && ff_);
  EXPECT_PRED2(FormulaEqual, f1_, tt_ && f1_);
  EXPECT_PRED2(FormulaEqual, f1_, f1_ && tt_);
}

TEST_F(SymbolicFormulaTest, And2) {
  EXPECT_EQ(f_and_.Evaluate(env1_), (1 + 1 > 0) && (1 * 1 < 5));
  EXPECT_EQ(f_and_.Evaluate(env2_), (3 + 4 > 0) && (3 * 4 < 5));
  EXPECT_EQ(f_and_.Evaluate(env3_), (-2 + -5 > 0) && (-2 * -5 < 5));
  EXPECT_EQ(f_and_.Evaluate(env4_), (-1 + -1 > 0) && (-1 * -1 < 5));

  EXPECT_EQ((x_ == 3 && y_ == 5).to_string(), "((x = 3) and (y = 5))");
  EXPECT_TRUE(is_conjunction(x_ == 3 && y_ == 5));
  EXPECT_TRUE(is_nary(x_ == 3 && y_ == 5));
}

TEST_F(SymbolicFormulaTest, And3) {
  // Flattening
  EXPECT_PRED2(FormulaEqual, f1_ && f2_ && f3_ && f4_,
               f1_ && f2_ && f3_ && f4_);
  EXPECT_PRED2(FormulaEqual, (f1_ && f2_) && (f3_ && f4_),
               f1_ && f2_ && f3_ && f4_);
  EXPECT_PRED2(FormulaEqual, f1_ && (f2_ && f3_) && f4_,
               f1_ && f2_ && f3_ && f4_);
  EXPECT_PRED2(FormulaEqual, f1_ && ((f2_ && f3_) && f4_),
               f1_ && f2_ && f3_ && f4_);
  // Remove duplicate
  EXPECT_PRED2(FormulaEqual, f1_ && f2_ && f1_, f1_ && f2_);
}

TEST_F(SymbolicFormulaTest, Or1) {
  EXPECT_PRED2(FormulaEqual, tt_, tt_ || tt_);
  EXPECT_PRED2(FormulaEqual, tt_, ff_ || tt_);
  EXPECT_PRED2(FormulaEqual, tt_, tt_ || ff_);
  EXPECT_PRED2(FormulaEqual, ff_, ff_ || ff_);
  EXPECT_PRED2(FormulaEqual, f1_, ff_ || f1_);
  EXPECT_PRED2(FormulaEqual, f1_, f1_ || ff_);
}

TEST_F(SymbolicFormulaTest, Or2) {
  EXPECT_EQ(f_or_.Evaluate(env1_), (1 + 1 > 0) || (1 * 1 < 5));
  EXPECT_EQ(f_or_.Evaluate(env2_), (3 + 4 > 0) || (3 * 4 < 5));
  EXPECT_EQ(f_or_.Evaluate(env3_), (-2 + -5 > 0) || (-2 * -5 < 5));
  EXPECT_EQ(f_or_.Evaluate(env4_), (-1 + -1 > 0) || (-1 * -1 < 5));

  EXPECT_EQ((x_ == 3 || y_ == 5).to_string(), "((x = 3) or (y = 5))");
  EXPECT_TRUE(is_disjunction(x_ == 3 || y_ == 5));
  EXPECT_TRUE(is_nary(x_ == 3 || y_ == 5));
}

TEST_F(SymbolicFormulaTest, Or3) {
  // Flattening
  EXPECT_PRED2(FormulaEqual, f1_ || f2_ || f3_ || f4_,
               f1_ || f2_ || f3_ || f4_);
  EXPECT_PRED2(FormulaEqual, (f1_ || f2_) || (f3_ || f4_),
               f1_ || f2_ || f3_ || f4_);
  EXPECT_PRED2(FormulaEqual, f1_ || (f2_ || f3_) || f4_,
               f1_ || f2_ || f3_ || f4_);
  EXPECT_PRED2(FormulaEqual, f1_ || ((f2_ || f3_) || f4_),
               f1_ || f2_ || f3_ || f4_);
  // Remove duplicate
  EXPECT_PRED2(FormulaEqual, f1_ || f2_ || f1_, f1_ || f2_);
}

TEST_F(SymbolicFormulaTest, Not1) {
  EXPECT_PRED2(FormulaEqual, ff_, !tt_);
  EXPECT_PRED2(FormulaEqual, tt_, !ff_);
  EXPECT_PRED2(FormulaEqual, tt_, !(!tt_));
  EXPECT_PRED2(FormulaEqual, ff_, !(!ff_));
  EXPECT_PRED2(FormulaEqual, !(x_ == 5), !(x_ == 5));
  EXPECT_PRED2(FormulaNotEqual, !(x_ == 5), !(x_ == 6));
}

TEST_F(SymbolicFormulaTest, Not2) {
  EXPECT_EQ(not_f_or_.Evaluate(env1_), !((1 + 1 > 0) || (1 * 1 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env2_), !((3 + 4 > 0) || (3 * 4 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env3_), !((-2 + -5 > 0) || (-2 * -5 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env4_), !((-1 + -1 > 0) || (-1 * -1 < 5)));

  EXPECT_EQ((!(x_ == 5)).to_string(), "!((x = 5))");
  EXPECT_TRUE(is_negation(!(x_ == 5)));
}

TEST_F(SymbolicFormulaTest, Forall1) {
  const Formula f1{forall({var_x_, var_y_}, x_ == y_)};
  const Formula f2{forall({var_x_, var_y_}, x_ == y_)};
  const Formula f3{forall({var_x_, var_y_}, x_ > y_)};
  EXPECT_PRED2(FormulaEqual, f1, f2);
  EXPECT_PRED2(FormulaNotEqual, f1, f3);
  EXPECT_PRED2(FormulaNotEqual, f2, f3);

  EXPECT_TRUE(all_of({f1, f2, f3}, is_forall));
}

TEST_F(SymbolicFormulaTest, Forall2) {
  const Formula f1{forall({var_x_, var_y_}, x_ == y_)};
  EXPECT_THROW(f1.Evaluate(), std::runtime_error);
}

TEST_F(SymbolicFormulaTest, GetFreeVariables) {
  const Formula f1{x_ + y_ > 0};
  const Formula f2{y_ * z_ < 5};
  const Formula f_or{f1 || f2};
  const Formula f_forall{forall({var_x_, var_y_}, f_or)};

  const Variables vars1{f1.GetFreeVariables()};  // {x_, y_}
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_TRUE(vars1.include(var_x_));
  EXPECT_TRUE(vars1.include(var_y_));

  const Variables vars2{f2.GetFreeVariables()};  // {y_, z_}
  EXPECT_EQ(vars2.size(), 2u);
  EXPECT_TRUE(vars2.include(var_y_));
  EXPECT_TRUE(vars2.include(var_z_));

  const Variables vars3{f_or.GetFreeVariables()};  // {x_, y_, z_}
  EXPECT_EQ(vars3.size(), 3u);
  EXPECT_TRUE(vars3.include(var_x_));
  EXPECT_TRUE(vars3.include(var_y_));
  EXPECT_TRUE(vars3.include(var_z_));

  const Variables vars4{f_forall.GetFreeVariables()};  // {z_}
  EXPECT_EQ(vars4.size(), 1u);
  EXPECT_TRUE(vars4.include(var_z_));

  const Variables vars5{(!f1).GetFreeVariables()};  // {x_, y_}
  EXPECT_EQ(vars5.size(), 2u);
  EXPECT_TRUE(vars5.include(var_x_));
  EXPECT_TRUE(vars5.include(var_y_));
}

TEST_F(SymbolicFormulaTest, ToString) {
  EXPECT_EQ(f1_.to_string(), "((x + y) > 0)");
  EXPECT_EQ(f2_.to_string(), "((x * y) < 5)");
  EXPECT_EQ(f_or_.to_string(), "(((x + y) > 0) or ((x * y) < 5))");
  EXPECT_EQ(f_forall_.to_string(),
            "forall({x, y}. (((x + y) > 0) or ((x * y) < 5)))");
  EXPECT_EQ(f_isnan_.to_string(), "isnan(NaN)");
}

TEST_F(SymbolicFormulaTest, IsTrue) {
  EXPECT_TRUE(is_true(tt_));
  EXPECT_FALSE(any_of({ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_true));
}

TEST_F(SymbolicFormulaTest, IsFalse) {
  EXPECT_TRUE(is_false(ff_));
  EXPECT_FALSE(any_of({tt_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_false));
}

TEST_F(SymbolicFormulaTest, IsEqualTo) {
  EXPECT_TRUE(is_equal_to(f_eq_));
  EXPECT_FALSE(any_of({tt_, ff_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_equal_to));
}

TEST_F(SymbolicFormulaTest, IsNotEqualTo) {
  EXPECT_TRUE(is_not_equal_to(f_neq_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_lt_, f_lte_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_not_equal_to));
}

TEST_F(SymbolicFormulaTest, IsLessThan) {
  EXPECT_TRUE(is_less_than(f_lt_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lte_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_less_than));
}

TEST_F(SymbolicFormulaTest, IsLessThanOrEqualTo) {
  EXPECT_TRUE(is_less_than_or_equal_to(f_lte_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_gt_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_less_than_or_equal_to));
}

TEST_F(SymbolicFormulaTest, IsGreaterThan) {
  EXPECT_TRUE(is_greater_than(f_gt_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gte_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_greater_than));
}

TEST_F(SymbolicFormulaTest, IsGreaterThanOrEqualTo) {
  EXPECT_TRUE(is_greater_than_or_equal_to(f_gte_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_and_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_greater_than_or_equal_to));
}

TEST_F(SymbolicFormulaTest, IsRelational) {
  EXPECT_TRUE(
      all_of({f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_}, is_relational));
  EXPECT_FALSE(any_of({tt_, ff_, f_and_, f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_relational));
}

TEST_F(SymbolicFormulaTest, IsConjunction) {
  EXPECT_TRUE(is_conjunction(f_and_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       f_or_, not_f_or_, f_forall_, f_isnan_},
                      is_conjunction));
}

TEST_F(SymbolicFormulaTest, IsDisjunction) {
  EXPECT_TRUE(is_disjunction(f_or_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       f_and_, not_f_or_, f_forall_, f_isnan_},
                      is_disjunction));
}

TEST_F(SymbolicFormulaTest, IsNary) {
  EXPECT_TRUE(all_of({f_and_, f_or_}, is_nary));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       not_f_or_, f_forall_, f_isnan_},
                      is_nary));
}

TEST_F(SymbolicFormulaTest, IsNegation) {
  EXPECT_TRUE(is_negation(not_f_or_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       f_and_, f_or_, f_forall_, f_isnan_},
                      is_negation));
}

TEST_F(SymbolicFormulaTest, IsForall) {
  EXPECT_TRUE(is_forall(f_forall_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       f_and_, f_or_, not_f_or_, f_isnan_},
                      is_forall));
}

TEST_F(SymbolicFormulaTest, IsIsnan) {
  EXPECT_TRUE(is_isnan(f_isnan_));
  EXPECT_FALSE(any_of({tt_, ff_, f_eq_, f_neq_, f_lt_, f_lte_, f_gt_, f_gte_,
                       f_and_, f_or_, not_f_or_, f_forall_},
                      is_isnan));
}

TEST_F(SymbolicFormulaTest, GetLhsExpression) {
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ == 0.0), e1_);
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ != 0.0), e1_);
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ < 0.0), e1_);
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ <= 0.0), e1_);
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ > 0.0), e1_);
  EXPECT_PRED2(ExprEqual, get_lhs_expression(e1_ >= 0.0), e1_);
}

TEST_F(SymbolicFormulaTest, GetRhsExpression) {
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 == e1_), e1_);
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 != e1_), e1_);
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 < e1_), e1_);
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 <= e1_), e1_);
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 > e1_), e1_);
  EXPECT_PRED2(ExprEqual, get_rhs_expression(0.0 >= e1_), e1_);
}

TEST_F(SymbolicFormulaTest, GetOperandsConjunction) {
  const set<Formula> formulas{get_operands(f1_ && f2_ && f3_ && f4_)};
  EXPECT_EQ(formulas.size(), 4u);
  EXPECT_EQ(formulas.count(f1_), 1u);
  EXPECT_EQ(formulas.count(f2_), 1u);
  EXPECT_EQ(formulas.count(f3_), 1u);
  EXPECT_EQ(formulas.count(f4_), 1u);
}

TEST_F(SymbolicFormulaTest, GetOperandsDisjunction) {
  const set<Formula> formulas{get_operands(f1_ || f2_ || f3_ || f4_)};
  EXPECT_EQ(formulas.size(), 4u);
  EXPECT_EQ(formulas.count(f1_), 1u);
  EXPECT_EQ(formulas.count(f2_), 1u);
  EXPECT_EQ(formulas.count(f3_), 1u);
  EXPECT_EQ(formulas.count(f4_), 1u);
}

TEST_F(SymbolicFormulaTest, GetOperand) {
  EXPECT_PRED2(FormulaEqual, get_operand(!f1_), f1_);
}

TEST_F(SymbolicFormulaTest, GetQuantifiedVariables) {
  const Variables vars{var_x_, var_y_};
  const Formula f{x_ + y_ > z_};
  const Formula f_forall{forall(vars, f)};
  EXPECT_EQ(get_quantified_variables(f_forall), vars);
}

TEST_F(SymbolicFormulaTest, GetQuantifiedFormula) {
  const Variables vars{var_x_, var_y_};
  const Formula f{x_ + y_ > z_};
  const Formula f_forall{forall(vars, f)};
  EXPECT_PRED2(FormulaEqual, get_quantified_formula(f_forall), f);
}

// Confirm that formulas compile (and pass) Drake's assert-like checks.
TEST_F(SymbolicFormulaTest, DrakeAssert) {
  Formula mutable_f{x_ > 0};

  DRAKE_ASSERT(f1_);
  DRAKE_ASSERT(f2_);
  DRAKE_ASSERT(f_and_);
  DRAKE_ASSERT(f_or_);
  DRAKE_ASSERT(not_f_or_);
  DRAKE_ASSERT(f_forall_);
  DRAKE_ASSERT(f_isnan_);
  DRAKE_ASSERT(mutable_f);

  DRAKE_DEMAND(f1_);
  DRAKE_DEMAND(f2_);
  DRAKE_DEMAND(f_and_);
  DRAKE_DEMAND(f_or_);
  DRAKE_DEMAND(not_f_or_);
  DRAKE_DEMAND(f_forall_);
  DRAKE_DEMAND(f_isnan_);
  DRAKE_DEMAND(mutable_f);

  DRAKE_THROW_UNLESS(f1_);
  DRAKE_THROW_UNLESS(f2_);
  DRAKE_THROW_UNLESS(f_and_);
  DRAKE_THROW_UNLESS(f_or_);
  DRAKE_THROW_UNLESS(not_f_or_);
  DRAKE_THROW_UNLESS(f_forall_);
  DRAKE_THROW_UNLESS(f_isnan_);
  DRAKE_THROW_UNLESS(mutable_f);
}

// This test checks whether symbolic::Formula is compatible with
// std::unordered_set.
GTEST_TEST(FormulaTest, CompatibleWithUnorderedSet) {
  unordered_set<Formula, hash_value<Formula>> uset;
  uset.emplace(Formula::True());
  uset.emplace(Formula::True());
  uset.emplace(Formula::False());
  uset.emplace(Formula::False());
}

// This test checks whether symbolic::Formula is compatible with
// std::unordered_map.
GTEST_TEST(FormulaTest, CompatibleWithUnorderedMap) {
  unordered_map<Formula, Formula, hash_value<Formula>> umap;
  umap.emplace(Formula::True(), Formula::False());
  umap.emplace(Formula::False(), Formula::True());
}

// This test checks whether symbolic::Formula is compatible with
// std::set.
GTEST_TEST(FormulaTest, CompatibleWithSet) {
  set<Formula> set;
  set.emplace(Formula::True());
  set.emplace(Formula::True());
  set.emplace(Formula::False());
  set.emplace(Formula::False());
}

// This test checks whether symbolic::Formula is compatible with
// std::map.
GTEST_TEST(FormulaTest, CompatibleWithMap) {
  map<Formula, Formula> map;
  map.emplace(Formula::True(), Formula::False());
}

// This test checks whether symbolic::Formula is compatible with
// std::vector.
GTEST_TEST(FormulaTest, CompatibleWithVector) {
  vector<Formula> vec;
  vec.push_back(Formula::True());
}

GTEST_TEST(FormulaTest, NoThrowMoveConstructible) {
  // Make sure that symbolic::Formula is nothrow move-constructible so that
  // it can be moved (not copied) when a STL container (i.e. vector<Formula>)
  // is resized.
  EXPECT_TRUE(std::is_nothrow_move_constructible<Formula>::value);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
