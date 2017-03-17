#include "drake/geometry/type_safe_int_id.h"

#include <sstream>
#include <unordered_set>

#include "gtest/gtest.h"

namespace drake {
namespace geometry {
namespace  {

// Creates various dummy index types to test.
using std::stringstream;
using std::unordered_set;
using AId = TypeSafeIntId<class ATag>;
using BId = TypeSafeIntId<class BTag>;

// These tests confirm that *compilable* functionality behaves correctly.

// Verifies the constructor -- both explicit constructor and copy constructor.
// This also implicitly tests the value() method.
GTEST_TEST(TypeSafeIntId, Constructor) {
  AId a1{2};
  ASSERT_EQ(a1.value(), 2);
  AId a2(a1);
  ASSERT_EQ(a2.value(), 2);
};

// Confirms that assignment behaves correctly. This also implicitly tests
// equality and inequality.
GTEST_TEST(TypeSafeIntId, AssignmentAndComparison) {
  AId a1(1);
  AId a2(2);
  ASSERT_NE(a1, a2);
  a2 = a1;
  ASSERT_EQ(a1, a2);
}

// Confirms that frame ids are configured to serve as unique keys in
// STL containers.
GTEST_TEST(TypeSafeIntId, ServeAsMapKey) {
  unordered_set<AId> ids;
  EXPECT_EQ(ids.size(), 0);
  ids.insert(AId(0));
  EXPECT_EQ(ids.size(), 1);
  ids.insert(AId(1));
  EXPECT_EQ(ids.size(), 2);
  ids.insert(AId(0));
  EXPECT_EQ(ids.size(), 2);
}

// Tests the streaming behavior.
GTEST_TEST(TypeSafeIntId, StreamOperator) {
  stringstream ss;
  AId a(1);
  ss << a;
  EXPECT_EQ(ss.str(), "1");
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
GTEST_TEST(TypeSafeIntId, OP_NAME ## OperatorAvailiblity) { \
  EXPECT_FALSE((has_ ## OP_NAME<AId, BId>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AId, AId>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, int>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, size_t>())); \
  EXPECT_FALSE((has_ ## OP_NAME<AId, int64_t>())); \
}
BINARY_TEST(==, Equals)
BINARY_TEST(!=, NotEquals)
BINARY_TEST(=, Assignment)


// This tests that an identifier can't be cast into anything else.
template <typename ID, typename U,
          typename = decltype(static_cast<U>(GenerateLValue<ID>().get_thing()))>
bool can_cast_helper(int) { return true; }
template <typename T, typename U>
bool can_cast_helper(...) { return false; }
template <typename T, typename U>
bool can_cast() { return can_cast_helper<T, U>(1); }

GTEST_TEST(TypeSafeIntId, ConstructorAvailability) {
  // This only tests another id type and some reasonable int tests. This assumes
  // no one will attempt to cast to anything more exotic.
  EXPECT_FALSE((can_cast<AId, BId>()));
  EXPECT_FALSE((can_cast<AId, int>()));
  EXPECT_FALSE((can_cast<AId, size_t>()));
  EXPECT_FALSE((can_cast<AId, int64_t>()));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
