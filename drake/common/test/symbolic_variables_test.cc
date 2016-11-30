#include "drake/common/symbolic_variables.h"

#include "gtest/gtest.h"

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

// Provides common variables that are used by the following tests.
class SymbolicVariablesTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
  const Variable v_{"v"};
};

TEST_F(SymbolicVariablesTest, HashEq) {
  const Variables vars1{x_, y_, z_};
  const Variables vars2{z_, y_, x_};
  EXPECT_EQ(vars1.get_hash(), vars2.get_hash());
  EXPECT_EQ(vars1, vars2);
}

TEST_F(SymbolicVariablesTest, InsertSize) {
  Variables vars1{x_, y_, z_};
  EXPECT_EQ(vars1.size(), 3u);

  vars1.insert(x_);
  vars1.insert(y_);
  vars1.insert(z_);
  EXPECT_EQ(vars1.size(), 3u);
}

TEST_F(SymbolicVariablesTest, Plus) {
  Variables vars1{x_, y_, z_};
  EXPECT_EQ(vars1.size(), 3u);
  EXPECT_TRUE(vars1.include(x_));
  EXPECT_TRUE(vars1.include(y_));
  EXPECT_TRUE(vars1.include(z_));

  vars1 = vars1 + x_;
  vars1 += y_;
  vars1 += z_;
  EXPECT_EQ(vars1.size(), 3u);

  vars1 = vars1 + w_;
  EXPECT_EQ(vars1.size(), 4u);
  EXPECT_TRUE(vars1.include(w_));

  vars1 += v_;
  EXPECT_EQ(vars1.size(), 5u);
  EXPECT_TRUE(vars1.include(z_));
}

TEST_F(SymbolicVariablesTest, Erase) {
  Variables vars1{x_, y_, z_};
  Variables vars2{y_, z_, w_};
  EXPECT_EQ(vars1.size(), 3u);

  vars1.erase(y_);
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_FALSE(vars1.include(y_));

  vars1.erase(vars2);
  EXPECT_EQ(vars1.size(), 1u);
  EXPECT_TRUE(vars1.include(x_));
  EXPECT_FALSE(vars1.include(y_));
  EXPECT_FALSE(vars1.include(z_));
}

TEST_F(SymbolicVariablesTest, Minus) {
  Variables vars1{x_, y_, z_};
  EXPECT_EQ(vars1.size(), 3u);
  EXPECT_TRUE(vars1.include(x_));
  EXPECT_TRUE(vars1.include(y_));
  EXPECT_TRUE(vars1.include(z_));

  EXPECT_TRUE(vars1.include(x_));
  vars1 = vars1 - x_;
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_FALSE(vars1.include(x_));

  vars1 -= y_;
  EXPECT_FALSE(vars1.include(y_));
  vars1 -= z_;
  EXPECT_FALSE(vars1.include(z_));
  EXPECT_EQ(vars1.size(), 0u);

  Variables vars2{x_, y_, z_};
  const Variables vars3{y_, z_, w_};
  EXPECT_EQ(vars2.size(), 3u);
  vars2 -= vars3;
  EXPECT_EQ(vars2.size(), 1u);
  EXPECT_TRUE(vars2.include(x_));
}

TEST_F(SymbolicVariablesTest, IsSubsetOf) {
  const Variables vars1{x_, y_, z_, w_, v_};
  const Variables vars2{x_, y_};
  const Variables vars3{x_, y_, z_};
  const Variables vars4{z_, w_, v_};
  const Variables vars5{w_, v_};

  // vars1 = {x, y, z, w, v}
  EXPECT_TRUE(vars1.IsSubsetOf(vars1));
  EXPECT_FALSE(vars1.IsSubsetOf(vars2));
  EXPECT_FALSE(vars1.IsSubsetOf(vars3));
  EXPECT_FALSE(vars1.IsSubsetOf(vars4));
  EXPECT_FALSE(vars1.IsSubsetOf(vars5));

  // vars2 = {x, y}
  EXPECT_TRUE(vars2.IsSubsetOf(vars1));
  EXPECT_TRUE(vars2.IsSubsetOf(vars2));
  EXPECT_TRUE(vars2.IsSubsetOf(vars3));
  EXPECT_FALSE(vars2.IsSubsetOf(vars4));
  EXPECT_FALSE(vars2.IsSubsetOf(vars5));

  // vars3 = {x_, y_, z_}
  EXPECT_TRUE(vars3.IsSubsetOf(vars1));
  EXPECT_FALSE(vars3.IsSubsetOf(vars2));
  EXPECT_TRUE(vars3.IsSubsetOf(vars3));
  EXPECT_FALSE(vars3.IsSubsetOf(vars4));
  EXPECT_FALSE(vars3.IsSubsetOf(vars5));

  // vars4 = {z, w, v}
  EXPECT_TRUE(vars4.IsSubsetOf(vars1));
  EXPECT_FALSE(vars4.IsSubsetOf(vars2));
  EXPECT_FALSE(vars4.IsSubsetOf(vars3));
  EXPECT_TRUE(vars4.IsSubsetOf(vars4));
  EXPECT_FALSE(vars4.IsSubsetOf(vars5));

  // vars5 = {w, v}
  EXPECT_TRUE(vars5.IsSubsetOf(vars1));
  EXPECT_FALSE(vars5.IsSubsetOf(vars2));
  EXPECT_FALSE(vars5.IsSubsetOf(vars3));
  EXPECT_TRUE(vars5.IsSubsetOf(vars4));
  EXPECT_TRUE(vars5.IsSubsetOf(vars5));
}

TEST_F(SymbolicVariablesTest, IsStrictSubsetOf) {
  const Variables vars1{x_, y_, z_, w_, v_};
  const Variables vars2{x_, y_};
  const Variables vars3{x_, y_, z_};
  const Variables vars4{z_, w_, v_};
  const Variables vars5{w_, v_};

  // vars1 = {x, y, z, w, v}
  EXPECT_FALSE(vars1.IsStrictSubsetOf(vars1));
  EXPECT_FALSE(vars1.IsStrictSubsetOf(vars2));
  EXPECT_FALSE(vars1.IsStrictSubsetOf(vars3));
  EXPECT_FALSE(vars1.IsStrictSubsetOf(vars4));
  EXPECT_FALSE(vars1.IsStrictSubsetOf(vars5));

  // vars2 = {x, y}
  EXPECT_TRUE(vars2.IsStrictSubsetOf(vars1));
  EXPECT_FALSE(vars2.IsStrictSubsetOf(vars2));
  EXPECT_TRUE(vars2.IsStrictSubsetOf(vars3));
  EXPECT_FALSE(vars2.IsStrictSubsetOf(vars4));
  EXPECT_FALSE(vars2.IsStrictSubsetOf(vars5));

  // vars3 = {x_, y_, z_}
  EXPECT_TRUE(vars3.IsStrictSubsetOf(vars1));
  EXPECT_FALSE(vars3.IsStrictSubsetOf(vars2));
  EXPECT_FALSE(vars3.IsStrictSubsetOf(vars3));
  EXPECT_FALSE(vars3.IsStrictSubsetOf(vars4));
  EXPECT_FALSE(vars3.IsStrictSubsetOf(vars5));

  // vars4 = {z, w, v}
  EXPECT_TRUE(vars4.IsStrictSubsetOf(vars1));
  EXPECT_FALSE(vars4.IsStrictSubsetOf(vars2));
  EXPECT_FALSE(vars4.IsStrictSubsetOf(vars3));
  EXPECT_FALSE(vars4.IsStrictSubsetOf(vars4));
  EXPECT_FALSE(vars4.IsStrictSubsetOf(vars5));

  // vars5 = {w, v}
  EXPECT_TRUE(vars5.IsStrictSubsetOf(vars1));
  EXPECT_FALSE(vars5.IsStrictSubsetOf(vars2));
  EXPECT_FALSE(vars5.IsStrictSubsetOf(vars3));
  EXPECT_TRUE(vars5.IsStrictSubsetOf(vars4));
  EXPECT_FALSE(vars5.IsStrictSubsetOf(vars5));
}

TEST_F(SymbolicVariablesTest, IsSuperSetOf) {
  const Variables vars1{x_, y_, z_, w_, v_};
  const Variables vars2{x_, y_};
  const Variables vars3{x_, y_, z_};
  const Variables vars4{z_, w_, v_};
  const Variables vars5{w_, v_};

  // vars1 = {x, y, z, w, v}
  EXPECT_TRUE(vars1.IsSupersetOf(vars1));
  EXPECT_TRUE(vars1.IsSupersetOf(vars2));
  EXPECT_TRUE(vars1.IsSupersetOf(vars3));
  EXPECT_TRUE(vars1.IsSupersetOf(vars4));
  EXPECT_TRUE(vars1.IsSupersetOf(vars5));

  // vars2 = {x, y}
  EXPECT_FALSE(vars2.IsSupersetOf(vars1));
  EXPECT_TRUE(vars2.IsSupersetOf(vars2));
  EXPECT_FALSE(vars2.IsSupersetOf(vars3));
  EXPECT_FALSE(vars2.IsSupersetOf(vars4));
  EXPECT_FALSE(vars2.IsSupersetOf(vars5));

  // vars3 = {x_, y_, z_}
  EXPECT_FALSE(vars3.IsSupersetOf(vars1));
  EXPECT_TRUE(vars3.IsSupersetOf(vars2));
  EXPECT_TRUE(vars3.IsSupersetOf(vars3));
  EXPECT_FALSE(vars3.IsSupersetOf(vars4));
  EXPECT_FALSE(vars3.IsSupersetOf(vars5));

  // vars4 = {z, w, v}
  EXPECT_FALSE(vars4.IsSupersetOf(vars1));
  EXPECT_FALSE(vars4.IsSupersetOf(vars2));
  EXPECT_FALSE(vars4.IsSupersetOf(vars3));
  EXPECT_TRUE(vars4.IsSupersetOf(vars4));
  EXPECT_TRUE(vars4.IsSupersetOf(vars5));

  // vars5 = {w, v}
  EXPECT_FALSE(vars5.IsSupersetOf(vars1));
  EXPECT_FALSE(vars5.IsSupersetOf(vars2));
  EXPECT_FALSE(vars5.IsSupersetOf(vars3));
  EXPECT_FALSE(vars5.IsSupersetOf(vars4));
  EXPECT_TRUE(vars5.IsSupersetOf(vars5));
}

TEST_F(SymbolicVariablesTest, IsStrictSuperSetOf) {
  const Variables vars1{x_, y_, z_, w_, v_};
  const Variables vars2{x_, y_};
  const Variables vars3{x_, y_, z_};
  const Variables vars4{z_, w_, v_};
  const Variables vars5{w_, v_};

  // vars1 = {x, y, z, w, v}
  EXPECT_FALSE(vars1.IsStrictSupersetOf(vars1));
  EXPECT_TRUE(vars1.IsStrictSupersetOf(vars2));
  EXPECT_TRUE(vars1.IsStrictSupersetOf(vars3));
  EXPECT_TRUE(vars1.IsStrictSupersetOf(vars4));
  EXPECT_TRUE(vars1.IsStrictSupersetOf(vars5));

  // vars2 = {x, y}
  EXPECT_FALSE(vars2.IsStrictSupersetOf(vars1));
  EXPECT_FALSE(vars2.IsStrictSupersetOf(vars2));
  EXPECT_FALSE(vars2.IsStrictSupersetOf(vars3));
  EXPECT_FALSE(vars2.IsStrictSupersetOf(vars4));
  EXPECT_FALSE(vars2.IsStrictSupersetOf(vars5));

  // vars3 = {x_, y_, z_}
  EXPECT_FALSE(vars3.IsStrictSupersetOf(vars1));
  EXPECT_TRUE(vars3.IsStrictSupersetOf(vars2));
  EXPECT_FALSE(vars3.IsStrictSupersetOf(vars3));
  EXPECT_FALSE(vars3.IsStrictSupersetOf(vars4));
  EXPECT_FALSE(vars3.IsStrictSupersetOf(vars5));

  // vars4 = {z, w, v}
  EXPECT_FALSE(vars4.IsStrictSupersetOf(vars1));
  EXPECT_FALSE(vars4.IsStrictSupersetOf(vars2));
  EXPECT_FALSE(vars4.IsStrictSupersetOf(vars3));
  EXPECT_FALSE(vars4.IsStrictSupersetOf(vars4));
  EXPECT_TRUE(vars4.IsStrictSupersetOf(vars5));

  // vars5 = {w, v}
  EXPECT_FALSE(vars5.IsStrictSupersetOf(vars1));
  EXPECT_FALSE(vars5.IsStrictSupersetOf(vars2));
  EXPECT_FALSE(vars5.IsStrictSupersetOf(vars3));
  EXPECT_FALSE(vars5.IsStrictSupersetOf(vars4));
  EXPECT_FALSE(vars5.IsStrictSupersetOf(vars5));
}

TEST_F(SymbolicVariablesTest, ToString) {
  const Variables vars0{};
  const Variables vars1{x_, y_, z_, w_, v_};
  const Variables vars2{x_, y_};
  const Variables vars3{x_, y_, z_};
  const Variables vars4{z_, w_, v_};
  const Variables vars5{w_, v_};

  EXPECT_EQ(vars0.to_string(), "{}");
  EXPECT_EQ(vars1.to_string(), "{x, y, z, w, v}");
  EXPECT_EQ(vars2.to_string(), "{x, y}");
  EXPECT_EQ(vars3.to_string(), "{x, y, z}");
  EXPECT_EQ(vars4.to_string(), "{z, w, v}");
  EXPECT_EQ(vars5.to_string(), "{w, v}");
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
