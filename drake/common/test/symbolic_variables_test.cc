#include "drake/common/symbolic_variables.h"

#include "drake/common/symbolic_expression.h"
#include "gtest/gtest.h"

namespace drake {
namespace symbolic {
namespace core {
namespace test {
namespace {

using std::cerr;
using std::endl;
using std::to_string;

GTEST_TEST(SymVarsTest, hash_eq) {
  // - hash
  // - list constructors
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};

  Variables const vars1{x, y, z};
  Variables const vars2{z, y, x};

  EXPECT_EQ(vars1.get_hash(), vars2.get_hash());
  EXPECT_EQ(vars1, vars2);
}

GTEST_TEST(SymVarsTest, insert_size) {
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};

  Variables vars1{x, y, z};
  EXPECT_EQ(vars1.size(), 3u);

  vars1.insert(x);
  vars1.insert(y);
  vars1.insert(z);
  EXPECT_EQ(vars1.size(), 3u);
}

GTEST_TEST(SymVarsTest, operator_plus) {
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"z"};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

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

  Variables vars2 = {x, y, z};
  Variables vars3 = {y, z, w};
  EXPECT_EQ(vars2.size(), 3u);
  vars2 -= vars3;
  EXPECT_EQ(vars2.size(), 1u);
  EXPECT_TRUE(vars2.include(x));
}

GTEST_TEST(SymVarsTest, IsSubsetOf) {
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

  Variables const vars1 = {x, y, z, w, v};
  Variables const vars2 = {x, y};
  Variables const vars3 = {x, y, z};
  Variables const vars4 = {z, w, v};
  Variables const vars5 = {w, v};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

  Variables const vars1 = {x, y, z, w, v};
  Variables const vars2 = {x, y};
  Variables const vars3 = {x, y, z};
  Variables const vars4 = {z, w, v};
  Variables const vars5 = {w, v};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

  Variables const vars1 = {x, y, z, w, v};
  Variables const vars2 = {x, y};
  Variables const vars3 = {x, y, z};
  Variables const vars4 = {z, w, v};
  Variables const vars5 = {w, v};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

  Variables const vars1 = {x, y, z, w, v};
  Variables const vars2 = {x, y};
  Variables const vars3 = {x, y, z};
  Variables const vars4 = {z, w, v};
  Variables const vars5 = {w, v};

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
  Variable const x{"x"};
  Variable const y{"y"};
  Variable const z{"z"};
  Variable const w{"w"};
  Variable const v{"v"};

  Variables const vars0{};
  Variables const vars1{x, y, z, w, v};
  Variables const vars2{x, y};
  Variables const vars3{x, y, z};
  Variables const vars4{z, w, v};
  Variables const vars5{w, v};

  EXPECT_EQ(to_string(vars0), "{}");
  EXPECT_EQ(to_string(vars1), "{x, y, z, w, v}");
  EXPECT_EQ(to_string(vars2), "{x, y}");
  EXPECT_EQ(to_string(vars3), "{x, y, z}");
  EXPECT_EQ(to_string(vars4), "{z, w, v}");
  EXPECT_EQ(to_string(vars5), "{w, v}");
}
}  // namespace
}  // namespace test
}  // namespace core
}  // namespace symbolic
}  // namespace drake
