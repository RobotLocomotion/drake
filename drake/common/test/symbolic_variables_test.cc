#include "drake/common/symbolic_variables.h"

#include "gtest/gtest.h"

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace symbolic {
namespace {

GTEST_TEST(SymVarsTest, hash_eq) {
  // - hash
  // - list constructors
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};

  const Variables vars1{x, y, z};
  const Variables vars2{z, y, x};

  EXPECT_EQ(vars1.get_hash(), vars2.get_hash());
  EXPECT_EQ(vars1, vars2);
}

GTEST_TEST(SymVarsTest, insert_size) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};

  Variables vars1{x, y, z};
  EXPECT_EQ(vars1.size(), 3u);

  vars1.insert(x);
  vars1.insert(y);
  vars1.insert(z);
  EXPECT_EQ(vars1.size(), 3u);
}

GTEST_TEST(SymVarsTest, operator_plus) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  Variables vars1{x, y, z};
  EXPECT_EQ(vars1.size(), 3u);
  EXPECT_TRUE(vars1.include(x));
  EXPECT_TRUE(vars1.include(y));
  EXPECT_TRUE(vars1.include(z));

  vars1 = vars1 + x;
  vars1 += y;
  vars1 += z;
  EXPECT_EQ(vars1.size(), 3u);

  vars1 = vars1 + w;
  EXPECT_EQ(vars1.size(), 4u);
  EXPECT_TRUE(vars1.include(w));

  vars1 += v;
  EXPECT_EQ(vars1.size(), 5u);
  EXPECT_TRUE(vars1.include(z));
}

GTEST_TEST(SymVarsTest, erase) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"z"};

  Variables vars1{x, y, z};
  Variables vars2{y, z, w};
  EXPECT_EQ(vars1.size(), 3u);

  vars1.erase(y);
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_FALSE(vars1.include(y));

  vars1.erase(vars2);
  EXPECT_EQ(vars1.size(), 1u);
  EXPECT_TRUE(vars1.include(x));
  EXPECT_FALSE(vars1.include(y));
  EXPECT_FALSE(vars1.include(z));
}

GTEST_TEST(SymVarsTest, operator_minus) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  Variables vars1{x, y, z};
  EXPECT_EQ(vars1.size(), 3u);
  EXPECT_TRUE(vars1.include(x));
  EXPECT_TRUE(vars1.include(y));
  EXPECT_TRUE(vars1.include(z));

  EXPECT_TRUE(vars1.include(x));
  vars1 = vars1 - x;
  EXPECT_EQ(vars1.size(), 2u);
  EXPECT_FALSE(vars1.include(x));

  vars1 -= y;
  EXPECT_FALSE(vars1.include(y));
  vars1 -= z;
  EXPECT_FALSE(vars1.include(z));
  EXPECT_EQ(vars1.size(), 0u);

  Variables vars2{x, y, z};
  const Variables vars3{y, z, w};
  EXPECT_EQ(vars2.size(), 3u);
  vars2 -= vars3;
  EXPECT_EQ(vars2.size(), 1u);
  EXPECT_TRUE(vars2.include(x));
}

GTEST_TEST(SymVarsTest, IsSubsetOf) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  const Variables vars1{x, y, z, w, v};
  const Variables vars2{x, y};
  const Variables vars3{x, y, z};
  const Variables vars4{z, w, v};
  const Variables vars5{w, v};

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

  // vars3 = {x, y, z}
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

GTEST_TEST(SymVarsTest, IsStrictSubsetOf) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  const Variables vars1{x, y, z, w, v};
  const Variables vars2{x, y};
  const Variables vars3{x, y, z};
  const Variables vars4{z, w, v};
  const Variables vars5{w, v};

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

  // vars3 = {x, y, z}
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

GTEST_TEST(SymVarsTest, IsSuperSetOf) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  const Variables vars1{x, y, z, w, v};
  const Variables vars2{x, y};
  const Variables vars3{x, y, z};
  const Variables vars4{z, w, v};
  const Variables vars5{w, v};

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

  // vars3 = {x, y, z}
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

GTEST_TEST(SymVarsTest, IsStrictSuperSetOf) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  const Variables vars1{x, y, z, w, v};
  const Variables vars2{x, y};
  const Variables vars3{x, y, z};
  const Variables vars4{z, w, v};
  const Variables vars5{w, v};

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

  // vars3 = {x, y, z}
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

GTEST_TEST(SymVarsTest, output_operator) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  const Variable w{"w"};
  const Variable v{"v"};

  const Variables vars0{};
  const Variables vars1{x, y, z, w, v};
  const Variables vars2{x, y};
  const Variables vars3{x, y, z};
  const Variables vars4{z, w, v};
  const Variables vars5{w, v};

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
