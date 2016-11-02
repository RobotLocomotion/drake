#include "drake/common/symbolic_formula.h"

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/common/symbolic_environment.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {
namespace {

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
  const Formula f_and_{f1_ && f2_};
  const Formula f_or_{f1_ || f2_};
  const Formula not_f_or_{!f_or_};
  const Formula f_forall_{forall({var_x_, var_y_}, f_or_)};

  const Environment env1_{{var_x_, 1}, {var_y_, 1}};
  const Environment env2_{{var_x_, 3}, {var_y_, 4}};
  const Environment env3_{{var_x_, -2}, {var_y_, -5}};
  const Environment env4_{{var_x_, -1}, {var_y_, -1}};
};

TEST_F(SymbolicFormulaTest, True) { EXPECT_TRUE(Formula::True().Evaluate()); }

TEST_F(SymbolicFormulaTest, False) {
  EXPECT_FALSE(Formula::False().Evaluate());
}

TEST_F(SymbolicFormulaTest, EqualTo1) {
  const Formula f_eq = x_ == y_;
  const Formula f_ne = x_ != y_;
  const Formula f_lt = x_ < y_;
  const Formula f_le = x_ <= y_;
  const Formula f_gt = x_ > y_;
  const Formula f_ge = x_ >= y_;

  EXPECT_TRUE(f_eq.EqualTo(f_eq));
  EXPECT_FALSE(f_eq.EqualTo(f_ne));
  EXPECT_FALSE(f_eq.EqualTo(f_lt));
  EXPECT_FALSE(f_eq.EqualTo(f_le));
  EXPECT_FALSE(f_eq.EqualTo(f_gt));
  EXPECT_FALSE(f_eq.EqualTo(f_ge));

  EXPECT_FALSE(f_ne.EqualTo(f_eq));
  EXPECT_TRUE(f_ne.EqualTo(f_ne));
  EXPECT_FALSE(f_ne.EqualTo(f_lt));
  EXPECT_FALSE(f_ne.EqualTo(f_le));
  EXPECT_FALSE(f_ne.EqualTo(f_gt));
  EXPECT_FALSE(f_ne.EqualTo(f_ge));

  EXPECT_FALSE(f_lt.EqualTo(f_eq));
  EXPECT_FALSE(f_lt.EqualTo(f_ne));
  EXPECT_TRUE(f_lt.EqualTo(f_lt));
  EXPECT_FALSE(f_lt.EqualTo(f_le));
  EXPECT_FALSE(f_lt.EqualTo(f_gt));
  EXPECT_FALSE(f_lt.EqualTo(f_ge));

  EXPECT_FALSE(f_le.EqualTo(f_eq));
  EXPECT_FALSE(f_le.EqualTo(f_ne));
  EXPECT_FALSE(f_le.EqualTo(f_lt));
  EXPECT_TRUE(f_le.EqualTo(f_le));
  EXPECT_FALSE(f_le.EqualTo(f_gt));
  EXPECT_FALSE(f_le.EqualTo(f_ge));

  EXPECT_FALSE(f_gt.EqualTo(f_eq));
  EXPECT_FALSE(f_gt.EqualTo(f_ne));
  EXPECT_FALSE(f_gt.EqualTo(f_lt));
  EXPECT_FALSE(f_gt.EqualTo(f_le));
  EXPECT_TRUE(f_gt.EqualTo(f_gt));
  EXPECT_FALSE(f_gt.EqualTo(f_ge));

  EXPECT_FALSE(f_ge.EqualTo(f_eq));
  EXPECT_FALSE(f_ge.EqualTo(f_ne));
  EXPECT_FALSE(f_ge.EqualTo(f_lt));
  EXPECT_FALSE(f_ge.EqualTo(f_le));
  EXPECT_FALSE(f_ge.EqualTo(f_gt));
  EXPECT_TRUE(f_ge.EqualTo(f_ge));
}

TEST_F(SymbolicFormulaTest, EqualTo2) {
  const Formula f1 = x_ == y_;
  const Formula f2 = y_ > z_;

  const Formula f_and = f1 && f2;
  const Formula f_or = f1 || f2;
  const Formula f_not = !f1;

  EXPECT_TRUE(f_and.EqualTo(f_and));
  EXPECT_FALSE(f_and.EqualTo(f_or));
  EXPECT_FALSE(f_and.EqualTo(f_not));

  EXPECT_FALSE(f_or.EqualTo(f_and));
  EXPECT_TRUE(f_or.EqualTo(f_or));
  EXPECT_FALSE(f_or.EqualTo(f_not));

  EXPECT_FALSE(f_not.EqualTo(f_and));
  EXPECT_FALSE(f_not.EqualTo(f_or));
  EXPECT_TRUE(f_not.EqualTo(f_not));
}

TEST_F(SymbolicFormulaTest, EqualTo3) {
  const Formula f_forall1{forall({var_x_, var_y_}, f_or_)};
  const Formula f_forall2{forall({var_x_, var_y_, var_z_}, f_or_)};
  const Formula f_forall3{forall({var_x_, var_y_}, f_and_)};
  const Formula f_forall4{forall({var_x_, var_y_, var_z_}, f_and_)};

  EXPECT_TRUE(f_forall1.EqualTo(f_forall1));
  EXPECT_FALSE(f_forall1.EqualTo(f_forall2));
  EXPECT_FALSE(f_forall1.EqualTo(f_forall3));
  EXPECT_FALSE(f_forall1.EqualTo(f_forall4));

  EXPECT_FALSE(f_forall2.EqualTo(f_forall1));
  EXPECT_TRUE(f_forall2.EqualTo(f_forall2));
  EXPECT_FALSE(f_forall2.EqualTo(f_forall3));
  EXPECT_FALSE(f_forall2.EqualTo(f_forall4));

  EXPECT_FALSE(f_forall3.EqualTo(f_forall1));
  EXPECT_FALSE(f_forall3.EqualTo(f_forall2));
  EXPECT_TRUE(f_forall3.EqualTo(f_forall3));
  EXPECT_FALSE(f_forall3.EqualTo(f_forall4));

  EXPECT_FALSE(f_forall4.EqualTo(f_forall1));
  EXPECT_FALSE(f_forall4.EqualTo(f_forall2));
  EXPECT_FALSE(f_forall4.EqualTo(f_forall3));
  EXPECT_TRUE(f_forall4.EqualTo(f_forall4));
}

TEST_F(SymbolicFormulaTest, Eq) {
  const Formula f1{e1_ == e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1_ == e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1_ == e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2_ == e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) == (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) == (2 + 3));
}

TEST_F(SymbolicFormulaTest, Neq) {
  const Formula f1{e1_ != e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1_ != e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1_ != e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::False()));
  const Formula f4{e2_ != e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::False()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) != (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) != (2 + 3));
}

TEST_F(SymbolicFormulaTest, Lt) {
  const Formula f1{e1_ < e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1_ < e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1_ < e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2_ < e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) < (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) < (2 + 3));
}

TEST_F(SymbolicFormulaTest, Gt) {
  const Formula f1{e1_ > e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1_ > e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1_ > e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2_ > e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) > (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) > (2 + 3));
}

TEST_F(SymbolicFormulaTest, Leq) {
  const Formula f1{e1_ <= e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1_ <= e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1_ <= e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2_ <= e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) <= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) <= (2 + 3));
}

TEST_F(SymbolicFormulaTest, Geq) {
  const Formula f1{e1_ >= e1_};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1_ >= e1_prime_};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1_ >= e3_};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2_ >= e3_};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) >= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) >= (2 + 3));
}

TEST_F(SymbolicFormulaTest, And1) {
  EXPECT_TRUE(tt_.EqualTo(tt_ && tt_));
  EXPECT_TRUE(ff_.EqualTo(ff_ && tt_));
  EXPECT_TRUE(ff_.EqualTo(tt_ && ff_));
  EXPECT_TRUE(ff_.EqualTo(ff_ && ff_));
}

TEST_F(SymbolicFormulaTest, And2) {
  EXPECT_EQ(f_and_.Evaluate(env1_), (1 + 1 > 0) && (1 * 1 < 5));
  EXPECT_EQ(f_and_.Evaluate(env2_), (3 + 4 > 0) && (3 * 4 < 5));
  EXPECT_EQ(f_and_.Evaluate(env3_), (-2 + -5 > 0) && (-2 * -5 < 5));
  EXPECT_EQ(f_and_.Evaluate(env4_), (-1 + -1 > 0) && (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Or2) {
  EXPECT_EQ(f_or_.Evaluate(env1_), (1 + 1 > 0) || (1 * 1 < 5));
  EXPECT_EQ(f_or_.Evaluate(env2_), (3 + 4 > 0) || (3 * 4 < 5));
  EXPECT_EQ(f_or_.Evaluate(env3_), (-2 + -5 > 0) || (-2 * -5 < 5));
  EXPECT_EQ(f_or_.Evaluate(env4_), (-1 + -1 > 0) || (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Not1) {
  EXPECT_TRUE(ff_.EqualTo(!tt_));
  EXPECT_TRUE(tt_.EqualTo(!ff_));
  EXPECT_TRUE(tt_.EqualTo(!(!tt_)));
  EXPECT_TRUE(ff_.EqualTo(!(!ff_)));
}

TEST_F(SymbolicFormulaTest, Not2) {
  EXPECT_EQ(not_f_or_.Evaluate(env1_), !((1 + 1 > 0) || (1 * 1 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env2_), !((3 + 4 > 0) || (3 * 4 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env3_), !((-2 + -5 > 0) || (-2 * -5 < 5)));
  EXPECT_EQ(not_f_or_.Evaluate(env4_), !((-1 + -1 > 0) || (-1 * -1 < 5)));
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
}

TEST_F(SymbolicFormulaTest, ToString) {
  EXPECT_EQ(f1_.to_string(), "((x + y) > 0)");
  EXPECT_EQ(f2_.to_string(), "((x * y) < 5)");
  EXPECT_EQ(f_or_.to_string(), "(((x + y) > 0) or ((x * y) < 5))");
  EXPECT_EQ(f_forall_.to_string(),
            "forall({x, y}. (((x + y) > 0) or ((x * y) < 5)))");
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
  DRAKE_ASSERT(mutable_f);

  DRAKE_DEMAND(f1_);
  DRAKE_DEMAND(f2_);
  DRAKE_DEMAND(f_and_);
  DRAKE_DEMAND(f_or_);
  DRAKE_DEMAND(not_f_or_);
  DRAKE_DEMAND(f_forall_);
  DRAKE_DEMAND(mutable_f);

  DRAKE_THROW_UNLESS(f1_);
  DRAKE_THROW_UNLESS(f2_);
  DRAKE_THROW_UNLESS(f_and_);
  DRAKE_THROW_UNLESS(f_or_);
  DRAKE_THROW_UNLESS(not_f_or_);
  DRAKE_THROW_UNLESS(f_forall_);
  DRAKE_THROW_UNLESS(mutable_f);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
