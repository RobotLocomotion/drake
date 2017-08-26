#include "drake/automotive/maliput/api/typed_id.h"

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

namespace drake {
namespace maliput {
namespace api {
namespace {


class C {};  // some random class

using CId = TypedId<C>;  // ID's specifically identifying Class C


GTEST_TEST(TypedIdTest, Construction) {
  EXPECT_NO_THROW(CId("x"));
  EXPECT_THROW(CId(""), std::runtime_error);
}


GTEST_TEST(TypedIdTest, IdentifiedType) {
  ::testing::StaticAssertTypeEq<CId::identified_type, C>();
}


GTEST_TEST(TypedIdTest, Accessors) {
  const CId dut("x");
  EXPECT_EQ(dut.string(), "x");
}


GTEST_TEST(TypedIdTest, Equality) {
  const CId dut_x1("x");
  const CId dut_x2("x");
  const CId dut_y("y");

  EXPECT_TRUE(dut_x1 == dut_x2);
  EXPECT_FALSE(dut_x1 != dut_x2);

  EXPECT_FALSE(dut_x1 == dut_y);
  EXPECT_TRUE(dut_x1 != dut_y);
}


GTEST_TEST(TypedIdTest, CopyingAndAssignment) {
  const CId dut1("x");
  const CId dut2(dut1);
  CId dut3("y");
  dut3 = dut1;

  EXPECT_TRUE(dut1 == dut2);
  EXPECT_TRUE(dut1 == dut3);
}


// Test usage with ordered/unordered sets.
template <typename T>
class TypedIdSetTest : public ::testing::Test {};

typedef ::testing::Types<std::set<CId>,
                         std::unordered_set<CId>> SetTypes;

TYPED_TEST_CASE(TypedIdSetTest, SetTypes);

TYPED_TEST(TypedIdSetTest, SetTypes) {
  TypeParam dut;

  dut.insert(CId("a"));
  dut.insert(CId("b"));
  dut.insert(CId("c"));
  // Insert a fresh, duplicate instance of "a".
  dut.insert(CId("a"));

  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut.count(CId("a")), 1);
  EXPECT_EQ(dut.count(CId("b")), 1);
  EXPECT_EQ(dut.count(CId("c")), 1);
}


// Test usage with ordered/unordered maps.
template <typename T>
class TypedIdMapTest : public ::testing::Test {};

typedef ::testing::Types<std::map<CId, int>,
                         std::unordered_map<CId, int>> MapTypes;

TYPED_TEST_CASE(TypedIdMapTest, MapTypes);

TYPED_TEST(TypedIdMapTest, MapTypes) {
  TypeParam dut;

  dut[CId("a")] = 1;
  dut[CId("b")] = 2;
  dut[CId("c")] = 3;
  // Insert a fresh, duplicate instance of "a".
  dut[CId("a")] = 5;

  EXPECT_EQ(dut.size(), 3);
  EXPECT_EQ(dut[CId("a")], 5);
  EXPECT_EQ(dut[CId("b")], 2);
  EXPECT_EQ(dut[CId("c")], 3);
}

}  // namespace
}  // namespace api
}  // namespace maliput
}  // namespace drake
