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
  const Variable var_x{"x"};
  const Variable var_y{"y"};
  const Variable var_z{"z"};
  const Expression x{var_x};
  const Expression y{var_y};
  const Expression z{var_z};

  const Formula tt{Formula::True()};
  const Formula ff{Formula::False()};
};

TEST_F(SymbolicFormulaTest, True) { EXPECT_TRUE(Formula::True().Evaluate()); }

TEST_F(SymbolicFormulaTest, False) {
  EXPECT_FALSE(Formula::False().Evaluate());
}

TEST_F(SymbolicFormulaTest, EqualTo) {}

TEST_F(SymbolicFormulaTest, Eq) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) == (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) == (2 + 3));
}

TEST_F(SymbolicFormulaTest, Neq) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) != (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) != (2 + 3));
}

TEST_F(SymbolicFormulaTest, Lt) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) < (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) < (2 + 3));
}

TEST_F(SymbolicFormulaTest, Gt) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) > (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) > (2 + 3));
}

TEST_F(SymbolicFormulaTest, Leq) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) <= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) <= (2 + 3));
}

TEST_F(SymbolicFormulaTest, Geq) {
  const Expression e1{x + y};
  const Expression e1_prime{x + y};
  const Expression e2{x - y};
  const Expression e3{x + z};

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

  const Environment env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) >= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) >= (2 + 3));
}

TEST_F(SymbolicFormulaTest, And1) {
  EXPECT_TRUE(tt.EqualTo(tt && tt));
  EXPECT_TRUE(ff.EqualTo(ff && tt));
  EXPECT_TRUE(ff.EqualTo(tt && ff));
  EXPECT_TRUE(ff.EqualTo(ff && ff));
}

TEST_F(SymbolicFormulaTest, And2) {
  const Formula f1{x + y > 0};
  const Formula f2{x * y < 5};
  const Formula f{f1 && f2};

  const Environment env1{{var_x, 1}, {var_y, 1}};
  const Environment env2{{var_x, 3}, {var_y, 4}};
  const Environment env3{{var_x, -2}, {var_y, -5}};
  const Environment env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) && (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) && (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) && (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) && (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Or2) {
  const Formula f1{x + y > 0};
  const Formula f2{x * y < 5};
  const Formula f{f1 || f2};

  const Environment env1{{var_x, 1}, {var_y, 1}};
  const Environment env2{{var_x, 3}, {var_y, 4}};
  const Environment env3{{var_x, -2}, {var_y, -5}};
  const Environment env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) || (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) || (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) || (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) || (-1 * -1 < 5));
}

TEST_F(SymbolicFormulaTest, Not1) {
  EXPECT_TRUE(ff.EqualTo(!tt));
  EXPECT_TRUE(tt.EqualTo(!ff));
  EXPECT_TRUE(tt.EqualTo(!(!tt)));
  EXPECT_TRUE(ff.EqualTo(!(!ff)));
}

TEST_F(SymbolicFormulaTest, Not2) {
  const Formula f1{x + y > 0};
  const Formula f2{x * y < 5};
  const Formula f{f1 || f2};
  const Formula not_f{!f};

  const Environment env1{{var_x, 1}, {var_y, 1}};
  const Environment env2{{var_x, 3}, {var_y, 4}};
  const Environment env3{{var_x, -2}, {var_y, -5}};
  const Environment env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(not_f.Evaluate(env1), !((1 + 1 > 0) || (1 * 1 < 5)));
  EXPECT_EQ(not_f.Evaluate(env2), !((3 + 4 > 0) || (3 * 4 < 5)));
  EXPECT_EQ(not_f.Evaluate(env3), !((-2 + -5 > 0) || (-2 * -5 < 5)));
  EXPECT_EQ(not_f.Evaluate(env4), !((-1 + -1 > 0) || (-1 * -1 < 5)));
}

TEST_F(SymbolicFormulaTest, GetFreeVariables) {
  const Formula f1{x + y > 0};
  const Formula f2{y * z < 5};
  const Formula f3{f1 || f2};
  const Formula f4{forall({var_x, var_y}, f3)};

  const Variables vars1{f1.GetFreeVariables()};  // {x, y}
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_TRUE(vars1.include(var_x));
  EXPECT_TRUE(vars1.include(var_y));

  const Variables vars2{f2.GetFreeVariables()};  // {y, z}
  EXPECT_EQ(vars2.size(), 2u);
  EXPECT_TRUE(vars2.include(var_y));
  EXPECT_TRUE(vars2.include(var_z));

  const Variables vars3{f3.GetFreeVariables()};  // {x, y, z}
  EXPECT_EQ(vars3.size(), 3u);
  EXPECT_TRUE(vars3.include(var_x));
  EXPECT_TRUE(vars3.include(var_y));
  EXPECT_TRUE(vars3.include(var_z));

  const Variables vars4{f4.GetFreeVariables()};  // {z}
  EXPECT_EQ(vars4.size(), 1u);
  EXPECT_TRUE(vars4.include(var_z));
}

TEST_F(SymbolicFormulaTest, ToString) {
  const Formula f1{x + y > 0};
  const Formula f2{y * z < 5};
  const Formula f3{f1 || f2};
  const Formula f4{forall({var_x, var_y}, f3)};

  EXPECT_EQ(f1.to_string(), "((x + y) > 0)");
  EXPECT_EQ(f2.to_string(), "((y * z) < 5)");
  EXPECT_EQ(f3.to_string(), "(((x + y) > 0) or ((y * z) < 5))");
  EXPECT_EQ(f4.to_string(), "forall({x, y}. (((x + y) > 0) or ((y * z) < 5)))");
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
