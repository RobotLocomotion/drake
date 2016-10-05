#include "drake/common/symbolic_formula.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "drake/common/symbolic_expression.h"
#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace core {
namespace test {
namespace {

using std::cerr;
using std::endl;
using std::equal_to;
using std::ostringstream;
using std::string;
using std::to_string;
using std::runtime_error;

GTEST_TEST(SymFormulaTest, True) { EXPECT_TRUE(Formula::True().Evaluate()); }

GTEST_TEST(SymFormulaTest, False) { EXPECT_FALSE(Formula::False().Evaluate()); }

GTEST_TEST(SymFormulaTest, Eq) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 == e1);
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  Formula const f2 = (e1 == e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  Formula const f3 = (e1 == e3);
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  Formula const f4 = (e2 == e3);
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) == (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) == (2 + 3));
}

GTEST_TEST(SymFormulaTest, Neq) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 != e1);
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  Formula const f2 = (e1 != e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  Formula const f3 = (e1 != e3);
  EXPECT_FALSE(f3.EqualTo(Formula::False()));
  Formula const f4 = (e2 != e3);
  EXPECT_FALSE(f4.EqualTo(Formula::False()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) != (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) != (2 + 3));
}

GTEST_TEST(SymFormulaTest, Lt) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 < e1);
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  Formula const f2 = (e1 < e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  Formula const f3 = (e1 < e3);
  EXPECT_FALSE(f3.EqualTo(Formula::False()));
  Formula const f4 = (e2 < e3);
  EXPECT_FALSE(f4.EqualTo(Formula::False()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) < (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) < (2 + 3));
}

GTEST_TEST(SymFormulaTest, Gt) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 > e1);
  EXPECT_TRUE(f1.EqualTo(Formula::False()));
  Formula const f2 = (e1 > e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::False()));
  Formula const f3 = (e1 > e3);
  EXPECT_FALSE(f3.EqualTo(Formula::False()));
  Formula const f4 = (e2 > e3);
  EXPECT_FALSE(f4.EqualTo(Formula::False()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) > (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) > (2 + 3));
}

GTEST_TEST(SymFormulaTest, Leq) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 <= e1);
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  Formula const f2 = (e1 <= e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  Formula const f3 = (e1 <= e3);
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  Formula const f4 = (e2 <= e3);
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) <= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) <= (2 + 3));
}

GTEST_TEST(SymFormulaTest, Geq) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};
  Expression const e1 = x + y;
  Expression const e1_ = x + y;
  Expression const e2 = x - y;
  Expression const e3 = x + z;

  Formula const f1 = (e1 >= e1);
  EXPECT_TRUE(f1.EqualTo(Formula::True()));
  Formula const f2 = (e1 >= e1_);
  EXPECT_TRUE(f2.EqualTo(Formula::True()));
  Formula const f3 = (e1 >= e3);
  EXPECT_FALSE(f3.EqualTo(Formula::True()));
  Formula const f4 = (e2 >= e3);
  EXPECT_FALSE(f4.EqualTo(Formula::True()));

  Environment const env{{var_x, 2}, {var_y, 3}, {var_z, 3}};
  EXPECT_EQ(f3.Evaluate(env), (2 + 3) >= (2 + 3));
  EXPECT_EQ(f4.Evaluate(env), (2 - 3) >= (2 + 3));
}

GTEST_TEST(SymFormulaTest, And1) {
  Formula const tt = Formula::True();
  Formula const ff = Formula::False();
  EXPECT_TRUE(tt.EqualTo(tt && tt));
  EXPECT_TRUE(ff.EqualTo(ff && tt));
  EXPECT_TRUE(ff.EqualTo(tt && ff));
  EXPECT_TRUE(ff.EqualTo(ff && ff));
}

GTEST_TEST(SymFormulaTest, And2) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Expression const x{var_x};
  Expression const y{var_y};

  Formula const f1 = x + y > 0;
  Formula const f2 = x * y < 5;
  Formula const f = f1 && f2;

  Environment const env1{{var_x, 1}, {var_y, 1}};
  Environment const env2{{var_x, 3}, {var_y, 4}};
  Environment const env3{{var_x, -2}, {var_y, -5}};
  Environment const env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) && (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) && (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) && (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) && (-1 * -1 < 5));
}

GTEST_TEST(SymFormulaTest, Or2) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Expression const x{var_x};
  Expression const y{var_y};

  Formula const f1 = x + y > 0;
  Formula const f2 = x * y < 5;
  Formula const f = f1 || f2;

  Environment const env1{{var_x, 1}, {var_y, 1}};
  Environment const env2{{var_x, 3}, {var_y, 4}};
  Environment const env3{{var_x, -2}, {var_y, -5}};
  Environment const env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(f.Evaluate(env1), (1 + 1 > 0) || (1 * 1 < 5));
  EXPECT_EQ(f.Evaluate(env2), (3 + 4 > 0) || (3 * 4 < 5));
  EXPECT_EQ(f.Evaluate(env3), (-2 + -5 > 0) || (-2 * -5 < 5));
  EXPECT_EQ(f.Evaluate(env4), (-1 + -1 > 0) || (-1 * -1 < 5));
}

GTEST_TEST(SymFormulaTest, Not1) {
  Formula const tt = Formula::True();
  Formula const ff = Formula::False();
  EXPECT_TRUE(ff.EqualTo(!tt));
  EXPECT_TRUE(tt.EqualTo(!ff));
  EXPECT_TRUE(tt.EqualTo(!(!tt)));
  EXPECT_TRUE(ff.EqualTo(!(!ff)));
}

GTEST_TEST(SymFormulaTest, Not2) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Expression const x{var_x};
  Expression const y{var_y};

  Formula const f1 = x + y > 0;
  Formula const f2 = x * y < 5;
  Formula const f = f1 || f2;
  Formula const not_f = !f;

  Environment const env1{{var_x, 1}, {var_y, 1}};
  Environment const env2{{var_x, 3}, {var_y, 4}};
  Environment const env3{{var_x, -2}, {var_y, -5}};
  Environment const env4{{var_x, -1}, {var_y, -1}};

  EXPECT_EQ(not_f.Evaluate(env1), !((1 + 1 > 0) || (1 * 1 < 5)));
  EXPECT_EQ(not_f.Evaluate(env2), !((3 + 4 > 0) || (3 * 4 < 5)));
  EXPECT_EQ(not_f.Evaluate(env3), !((-2 + -5 > 0) || (-2 * -5 < 5)));
  EXPECT_EQ(not_f.Evaluate(env4), !((-1 + -1 > 0) || (-1 * -1 < 5)));
}

GTEST_TEST(SymFormulaTest, GetFreeVariables) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};

  Formula const f1 = x + y > 0;
  Formula const f2 = y * z < 5;
  Formula const f3 = f1 || f2;
  Formula const f4 = forall({var_x, var_y}, f3);

  Variables const vars1 = f1.GetFreeVariables();  // {x, y}
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_TRUE(vars1.include(var_x));
  EXPECT_TRUE(vars1.include(var_y));

  Variables const vars2 = f2.GetFreeVariables();  // {y, z}
  EXPECT_EQ(vars2.size(), 2u);
  EXPECT_TRUE(vars2.include(var_y));
  EXPECT_TRUE(vars2.include(var_z));

  Variables const vars3 = f3.GetFreeVariables();  // {x, y, z}
  EXPECT_EQ(vars3.size(), 3u);
  EXPECT_TRUE(vars3.include(var_x));
  EXPECT_TRUE(vars3.include(var_y));
  EXPECT_TRUE(vars3.include(var_z));

  Variables const vars4 = f4.GetFreeVariables();  // {z}
  EXPECT_EQ(vars4.size(), 1u);
  EXPECT_TRUE(vars4.include(var_z));
}

GTEST_TEST(SymFormulaTest, output_operator) {
  Variable const var_x{"x"};
  Variable const var_y{"y"};
  Variable const var_z{"z"};
  Expression const x{var_x};
  Expression const y{var_y};
  Expression const z{var_z};

  Formula const f1 = x + y > 0;
  Formula const f2 = y * z < 5;
  Formula const f3 = f1 || f2;
  Formula const f4 = forall({var_x, var_y}, f3);

  EXPECT_EQ(to_string(f1), "((x + y) > 0)");
  EXPECT_EQ(to_string(f2), "((y * z) < 5)");
  EXPECT_EQ(to_string(f3), "(((x + y) > 0) or ((y * z) < 5))");
  EXPECT_EQ(to_string(f4), "forall({x, y}. (((x + y) > 0) or ((y * z) < 5)))");
}
}  // namespace
}  // namespace test
}  // namespace core
}  // namespace symbolic
}  // namespace drake
