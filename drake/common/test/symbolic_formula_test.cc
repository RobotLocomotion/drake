#include "drake/common/symbolic_formula.h"

#include "gtest/gtest.h"

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
};

TEST_F(SymbolicFormulaTest, True) { EXPECT_TRUE(Formula::True().Evaluate()); }

TEST_F(SymbolicFormulaTest, False) {
  EXPECT_FALSE(Formula::False().Evaluate());
}

TEST_F(SymbolicFormulaTest, EqualTo) {}

TEST_F(SymbolicFormulaTest, Eq) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 == e1};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1 == e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1 == e3};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2 == e3};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) == (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) == (2 + 3));
}

TEST_F(SymbolicFormulaTest, Neq) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 != e1};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1 != e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1 != e3};
  EXPECT_FALSE(f3.EqualTo(Formula::False()));
  const Formula f4{e2 != e3};
  EXPECT_FALSE(f4.EqualTo(Formula::False()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) != (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) != (2 + 3));
}

TEST_F(SymbolicFormulaTest, Lt) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 < e1};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1 < e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1 < e3};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2 < e3};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) < (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) < (2 + 3));
}

TEST_F(SymbolicFormulaTest, Gt) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 > e1};
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  EXPECT_FALSE(f1.EqualTo(Formula::True()));
  const Formula f2{e1 > e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  EXPECT_FALSE(f2.EqualTo(Formula::True()));
  const Formula f3{e1 > e3};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2 > e3};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) > (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) > (2 + 3));
}

TEST_F(SymbolicFormulaTest, Leq) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 <= e1};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1 <= e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1 <= e3};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2 <= e3};
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  const Environment env{{var_x_, 2}, {var_y_, 3}, {var_z_, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) <= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) <= (2 + 3));
}

TEST_F(SymbolicFormulaTest, Geq) {
  const Expression e1{x_ + y_};
  const Expression e1_prime{x_ + y_};
  const Expression e2{x_ - y_};
  const Expression e3{x_ + z_};

  const Formula f1{e1 >= e1};
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  EXPECT_FALSE(f1.EqualTo(Formula::False()));
  const Formula f2{e1 >= e1_prime};
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  EXPECT_FALSE(f2.EqualTo(Formula::False()));
  const Formula f3{e1 >= e3};
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  const Formula f4{e2 >= e3};
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
  const Formula f1{x_ + y_ > 0};
  const Formula f2{x_ * y_ < 5};
  const Formula f{f1 && f2};

  const Environment env1{{var_x_, 1}, {var_y_, 1}};
  const Environment env2{{var_x_, 3}, {var_y_, 4}};
  const Environment env3{{var_x_, -2}, {var_y_, -5}};
  const Environment env4{{var_x_, -1}, {var_y_, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) && (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) && (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) && (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) && (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Or2) {
  const Formula f1{x_ + y_ > 0};
  const Formula f2{x_ * y_ < 5};
  const Formula f{f1 || f2};

  const Environment env1{{var_x_, 1}, {var_y_, 1}};
  const Environment env2{{var_x_, 3}, {var_y_, 4}};
  const Environment env3{{var_x_, -2}, {var_y_, -5}};
  const Environment env4{{var_x_, -1}, {var_y_, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) || (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) || (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) || (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) || (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Not1) {
  EXPECT_TRUE(ff_.EqualTo(!tt_));
  EXPECT_TRUE(tt_.EqualTo(!ff_));
  EXPECT_TRUE(tt_.EqualTo(!(!tt_)));
  EXPECT_TRUE(ff_.EqualTo(!(!ff_)));
}

TEST_F(SymbolicFormulaTest, Not2) {
  const Formula f1{x_ + y_ > 0};
  const Formula f2{x_ * y_ < 5};
  const Formula f{f1 || f2};
  const Formula not_f{!f};

  const Environment env1{{var_x_, 1}, {var_y_, 1}};
  const Environment env2{{var_x_, 3}, {var_y_, 4}};
  const Environment env3{{var_x_, -2}, {var_y_, -5}};
  const Environment env4{{var_x_, -1}, {var_y_, -1}};

  EXPECT_EQ(not_f.Evaluate(env1), !((1 + 1 > 0) || (1 * 1 < 5)));
  EXPECT_EQ(not_f.Evaluate(env2), !((3 + 4 > 0) || (3 * 4 < 5)));
  EXPECT_EQ(not_f.Evaluate(env3), !((-2 + -5 > 0) || (-2 * -5 < 5)));
  EXPECT_EQ(not_f.Evaluate(env4), !((-1 + -1 > 0) || (-1 * -1 < 5)));
}

TEST_F(SymbolicFormulaTest, GetFreeVariables) {
  const Formula f1{x_ + y_ > 0};
  const Formula f2{y_ * z_ < 5};
  const Formula f3{f1 || f2};
  const Formula f4{forall({var_x_, var_y_}, f3)};

  const Variables vars1{f1.GetFreeVariables()};  // {x_, y_}
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_TRUE(vars1.include(var_x_));
  EXPECT_TRUE(vars1.include(var_y_));

  const Variables vars2{f2.GetFreeVariables()};  // {y_, z_}
  EXPECT_EQ(vars2.size(), 2u);
  EXPECT_TRUE(vars2.include(var_y_));
  EXPECT_TRUE(vars2.include(var_z_));

  const Variables vars3{f3.GetFreeVariables()};  // {x_, y_, z_}
  EXPECT_EQ(vars3.size(), 3u);
  EXPECT_TRUE(vars3.include(var_x_));
  EXPECT_TRUE(vars3.include(var_y_));
  EXPECT_TRUE(vars3.include(var_z_));

  const Variables vars4{f4.GetFreeVariables()};  // {z_}
  EXPECT_EQ(vars4.size(), 1u);
  EXPECT_TRUE(vars4.include(var_z_));
}

TEST_F(SymbolicFormulaTest, ToString) {
  const Formula f1{x_ + y_ > 0};
  const Formula f2{y_ * z_ < 5};
  const Formula f3{f1 || f2};
  const Formula f4{forall({var_x_, var_y_}, f3)};

  EXPECT_EQ(f1.to_string(), "((x + y) > 0)");
  EXPECT_EQ(f2.to_string(), "((y * z) < 5)");
  EXPECT_EQ(f3.to_string(), "(((x + y) > 0) or ((y * z) < 5))");
  EXPECT_EQ(f4.to_string(), "forall({x, y}. (((x + y) > 0) or ((y * z) < 5)))");
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
