#include "drake/geometry/identifier.h"

#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "gtest/gtest.h"

namespace drake {
namespace geometry {
namespace {

// Creates various dummy index types to test.
using std::stringstream;
using std::unordered_set;
using std::unordered_map;
using AId = Identifier<class ATag>;
using BId = Identifier<class BTag>;

// The nature of gtest means these tests can be instantiated in any order.
// Any local instantiation of identifiers could lead to arbitrary values.
// This renders the tests that *care* about values very fragile.  The simplest
// way to account for this is to have these translation-unit-level globals that
// are constant w.r.t. execution order.  That is why these exist.
// For this to work, the following assumptions must be true:
//   1. It must run in a scope where these are the only invocations of
//      Identifier::get_new_id() (or, at the very least, these are the
//      first).
//   2. The Identifier::get_new_id() returns 0 with the first
//      invocation and each successive invocation increases the value by 1.
const AId a1 = AId::get_new_id();
const AId a2 = AId::get_new_id();
const AId a3 = AId::get_new_id();
const BId b = BId::get_new_id();

// These tests confirm that *compilable* functionality behaves correctly.

// Verifies the copy constructor. This implicitly tests the expected property
// of the get_new_id() factory method and the get_value() method.
GTEST_TEST(IdentifierTest, Constructor) {
  EXPECT_EQ(a1.get_value(), 1);
  EXPECT_EQ(a2.get_value(), 2);
  EXPECT_EQ(a3.get_value(), 3);
  AId temp(a2);
  EXPECT_EQ(temp.get_value(), 2);
  AId bad;
  EXPECT_FALSE(bad.is_valid());
  EXPECT_TRUE(a2.is_valid());
  // In Debug builds, attempting to acquire the value is an error.
#ifndef DRAKE_ASSERT_IS_DISARMED
  AId invalid;
  int64_t value = -1;
  ASSERT_DEATH({value = invalid.get_value();}, "");
#endif
};

// Confirms that assignment behaves correctly. This also implicitly tests
// equality and inequality.
GTEST_TEST(IdentifierTest, AssignmentAndComparison) {
  EXPECT_TRUE(a2 != a3);
  AId temp = a2;
  EXPECT_TRUE(temp == a2);
  temp = a3;
  EXPECT_TRUE(temp == a3);
  // In Debug builds, comparison of invalid ids is an error.
#ifndef DRAKE_ASSERT_IS_DISARMED
  AId invalid;
  bool result = true;
  ASSERT_DEATH({result = invalid == a1;}, "");
  ASSERT_DEATH({result = invalid != a1;}, "");
#endif
}

// Confirms that ids are configured to serve as unique keys in
// STL containers.
GTEST_TEST(IdentifierTest, ServeAsMapKey) {
  unordered_set<AId> ids;

  // This is a *different* id with the *same* value as a1. It should *not*
  // introduce a new value to the set.
  AId temp = a1;

  EXPECT_EQ(ids.size(), 0);
  ids.insert(a1);
  EXPECT_NE(ids.find(a1), ids.end());
  EXPECT_NE(ids.find(temp), ids.end());

  EXPECT_EQ(ids.size(), 1);
  ids.insert(a2);
  EXPECT_EQ(ids.size(), 2);

  ids.insert(temp);
  EXPECT_EQ(ids.size(), 2);

  EXPECT_EQ(ids.find(a3), ids.end());

  // In Debug builds, invalid identifiers cannot be used as keys.
#ifndef DRAKE_ASSERT_IS_DISARMED
  AId invalid;
  ASSERT_DEATH({ids.insert(invalid);}, "");
}

// Confirms that ids are configured to serve as values in STL containers.
GTEST_TEST(IdentifierTest, ServeAsMapValue) {
  unordered_map<BId, AId> ids;

  BId b1 = BId::get_new_id();
  BId b2 = BId::get_new_id();
  BId b3 = BId::get_new_id();
  ids.emplace(b1, a1);
  ids.emplace(b2, a2);
  EXPECT_EQ(ids.find(b3), ids.end());
  EXPECT_NE(ids.find(b2), ids.end());
  EXPECT_NE(ids.find(b1), ids.end());
  ids[b3] = a3;
  EXPECT_NE(ids.find(b3), ids.end());
#endif
}

// Tests the streaming behavior.
GTEST_TEST(IdentifierTest, StreamOperator) {
  stringstream ss;
  ss << a2;
  EXPECT_EQ(ss.str(), "2");
  // In Debug builds, writing the identifier value to a stream is an error.
#ifndef DRAKE_ASSERT_IS_DISARMED
  AId invalid;
  ASSERT_DEATH({ss << invalid;}, "");
#endif
}

// These tests confirm that behavior that *shouldn't* be compilable isn't.

// This class provides the compiler with an l-value to trigger compilation on.
template <class T>
struct GenerateLValue { T& get_thing(); };

// This code allows us to turn compile-time errors into run-time errors that
// we can incorporate in a unit test.  The macro simplifies the boilerplate.
// This macro confirms binary operations are *valid* between two ids of
// the same type, but invalid between an id and objects of *any* other type.
// (Although the space of "all other types", is sparsely sampled).
//
// The use is:
//    BINARY_TEST( op, op_name )
// It produces the templated method: has_op_name<T, U>(), which returns true
// if `t op u` is a valid operation for `T t` and `U u`.
//
// Examples of invocations:
//    op    |   op_name
// ---------+-------------
//   ==     |   equals
//    <     |   less_than
//    +     |   add
#define BINARY_TEST(OP, OP_NAME) \
template <typename T, typename U, \
    typename = decltype(GenerateLValue<T>().get_thing() OP \
                        GenerateLValue<U>().get_thing())> \
bool has_ ## OP_NAME ## _helper(int) { return true; } \
template <typename T, typename U> \
bool has_ ## OP_NAME ## _helper(...) { return false; } \
template <typename T, typename U> \
bool has_ ## OP_NAME() { return has_ ## OP_NAME ## _helper<T, U>(1); } \
GTEST_TEST(IdentifierTest, OP_NAME ## OperatorAvailiblity) { \
  EXPECT_FALSE((has_ ## OP_NAME<AId, BId>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AId, AId>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, int>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, size_t>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, int64_t>())); \
}
BINARY_TEST(==, Equals)
BINARY_TEST(!=, NotEquals)
BINARY_TEST(=, Assignment)

// Compile-time assertion
GTEST_TEST(IdentifierTest, Convertible) {
  static_assert(!std::is_convertible<AId, BId>::value,
                "Identifiers of different types should not be convertible.");
  static_assert(!std::is_convertible<AId, int>::value,
                "Identifiers should not be convertible to ints.");
  static_assert(!std::is_convertible<int, AId>::value,
                "Identifiers should not be convertible from ints");
}
}  // namespace
}  // namespace geometry
}  // namespace drake
