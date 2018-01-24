#include "drake/common/type_safe_index.h"

#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/unused.h"

namespace drake {
namespace common {
namespace {

using std::regex;
using std::regex_match;
using std::move;

#ifdef DRAKE_ASSERT_IS_DISARMED
// With assertion disarmed, expect no exception.
#define EXPECT_ERROR_MESSAGE_IF_ARMED(expression, reg_exp) \
do {\
  EXPECT_NO_THROW(expression); \
} while (0)

#else
// Helper macro for "expecting" an exception but *also* testing the error
// message against the provided regular expression.

#define EXPECT_ERROR_MESSAGE_IF_ARMED(expression, reg_exp) \
do {\
  try { \
    expression; \
    GTEST_NONFATAL_FAILURE_("\t" #expression \
        " failed to throw std::runtime_error."); \
  } catch (const std::runtime_error& err) { \
    auto matcher = [](const char* s, const char* re) { \
      return regex_match(s, regex(re)); }; \
    EXPECT_PRED2(matcher, err.what(), reg_exp); \
  } \
} while (0)
#endif

// Create dummy index types to exercise the functionality
using AIndex = TypeSafeIndex<class A>;
using BIndex = TypeSafeIndex<class B>;
using std::string;

// Verifies the constructor behavior -- in debug and release modes.
GTEST_TEST(TypeSafeIndex, Constructor) {
  AIndex index(1);
  EXPECT_EQ(index, 1);                      // This also tests operator==(int).
  AIndex index2(index);                     // Copy constructor.
  EXPECT_EQ(index2, 1);
  AIndex index3(move(index2));              // Move constructor.
  EXPECT_EQ(index3, 1);
  EXPECT_FALSE(index2.is_valid());

  // Construction with invalid index.
  AIndex invalid;                           // Default constructor.
  EXPECT_FALSE(invalid.is_valid());
  EXPECT_ERROR_MESSAGE_IF_ARMED(AIndex(-1),
                                "Explicitly constructing an invalid index.+");
  EXPECT_NO_THROW(unused(AIndex(invalid)));  // Copy construct invalid index.
}

// Verifies the constructor behavior -- in debug and release modes.
GTEST_TEST(TypeSafeIndex, IndexAssignment) {
  // Set up test initial conditions.
  AIndex index1(1);
  AIndex index2(2);
  EXPECT_NE(index2, index1);

  // Copy assignment[
  index2 = index1;
  EXPECT_EQ(index2, index1);

  // Move assignment.
  AIndex index3(2);
  EXPECT_NE(index3, index2);
  index3 = move(index1);
  EXPECT_EQ(index3, index2);
  EXPECT_FALSE(index1.is_valid());

  // Int assignment
  AIndex index4;
  index4 = 17;
  EXPECT_EQ(index4, 17);
  EXPECT_ERROR_MESSAGE_IF_ARMED(index4 = -3,
                                "Assigning an invalid int.+");

  // Assignment involving invalid indices.

  // Copy assign *to* invalid.
  {
    AIndex source(3);
    AIndex starts_invalid;
    EXPECT_NO_THROW(starts_invalid = source);
    EXPECT_EQ(source, 3);
    EXPECT_EQ(starts_invalid, source);
  }

  // Copy assign *from* invalid.
  {
    AIndex invalid;
    AIndex target(3);
    EXPECT_NO_THROW(target = invalid);
    EXPECT_FALSE(target.is_valid());
  }

  // Move assign *to* invalid.
  {
    AIndex invalid;
    AIndex target(2);
    EXPECT_TRUE(target.is_valid());
    EXPECT_NO_THROW(invalid = move(target));
    EXPECT_FALSE(target.is_valid());
    EXPECT_EQ(invalid, 2);
  }

  // Move assign *from* invalid.
  {
    AIndex invalid;
    AIndex target(3);
    EXPECT_NO_THROW(target = move(invalid));
    EXPECT_FALSE(target.is_valid());
    EXPECT_FALSE(invalid.is_valid());
  }
}

// Verifies implicit conversion from index to int.
GTEST_TEST(TypeSafeIndex, ConversionToInt) {
  AIndex index(4);
  int four = index;
  EXPECT_EQ(four, index);

  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(static_cast<int>(invalid)),
                                "Converting to an int.+");
}

// Tests valid comparisons of like-typed index instances.
GTEST_TEST(TypeSafeIndex, IndexComparisonOperators) {
  AIndex index1(5);
  AIndex index2(5);
  AIndex index3(7);
  // true-returning coverage.
  EXPECT_TRUE(index1 == index2);  // operator==
  EXPECT_TRUE(index1 != index3);  // operator!=
  EXPECT_TRUE(index1 <  index3);  // operator<
  EXPECT_TRUE(index1 <= index2);  // operator<=
  EXPECT_TRUE(index3 >  index1);  // operator>
  EXPECT_TRUE(index2 >= index1);  // operator>=
  // false-returning coverage.
  EXPECT_FALSE(index1 == index3);  // operator==
  EXPECT_FALSE(index1 != index2);  // operator!=
  EXPECT_FALSE(index3 <  index1);  // operator<
  EXPECT_FALSE(index3 <= index1);  // operator<=
  EXPECT_FALSE(index1 >  index3);  // operator>
  EXPECT_FALSE(index1 >= index3);  // operator>=
}

// Performs comparison operations on an invalid index. in Debug mode. these
// operations throw.
GTEST_TEST(TypeSafeIndex, InvalidIndexComparisonOperators) {
  AIndex valid(1);
  AIndex invalid;

  // Comparison operators.
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid == valid),
                                "Testing == with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid == invalid),
                                "Testing == with invalid RHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid != valid),
                                "Testing != with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid != invalid),
                                "Testing != with invalid RHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid < valid),
                                "Testing < with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid < invalid),
                                "Testing < with invalid RHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid <= valid),
                                "Testing <= with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid <= invalid),
                                "Testing <= with invalid RHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid > valid),
                                "Testing > with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid > invalid),
                                "Testing > with invalid RHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(invalid >= valid),
                                "Testing >= with invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(unused(valid >= invalid),
                                "Testing >= with invalid RHS.+");
}

// Tests the prefix increment behavior.
GTEST_TEST(TypeSafeIndex, PrefixIncrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_plus = ++index;
  EXPECT_EQ(index, AIndex(9));
  EXPECT_EQ(index_plus, AIndex(9));

  // In Debug builds, some increment operations will throw.
  // Overflow produces an invalid index.
  AIndex max_index(std::numeric_limits<int>::max());
  EXPECT_ERROR_MESSAGE_IF_ARMED(++max_index,
                                "Pre-incrementing produced an invalid index.+");
  // Increment invalid index.
  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(++invalid,
                                "Pre-incrementing an invalid index.+");
}

// Tests the postfix increment behavior.
GTEST_TEST(TypeSafeIndex, PostfixIncrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_plus = index++;
  EXPECT_EQ(index, AIndex(9));
  EXPECT_EQ(index_plus, AIndex(8));

  // In Debug builds, some increment operations will throw.
  // Overflow produces an invalid index.
  AIndex max_index(std::numeric_limits<int>::max());
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      max_index++, "Post-incrementing produced an invalid index.+");
  // Increment invalid index.
  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      invalid++, "Post-incrementing an invalid index.+");
}

// Tests the prefix decrement behavior.
GTEST_TEST(TypeSafeIndex, PrefixDecrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_minus = --index;
  EXPECT_EQ(index, AIndex(7));
  EXPECT_EQ(index_minus, AIndex(7));
  // In Debug builds decrements leading to a negative result will throw.
  AIndex about_to_be_negative_index(0);
  EXPECT_ERROR_MESSAGE_IF_ARMED(--about_to_be_negative_index,
                                "Pre-decrementing produced an invalid index.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      --AIndex(), "Pre-decrementing an invalid index.+");
}

// Tests the postfix decrement behavior.
GTEST_TEST(TypeSafeIndex, PostfixDecrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_minus = index--;
  EXPECT_EQ(index, AIndex(7));
  EXPECT_EQ(index_minus, AIndex(8));
  // In Debug builds decrements leading to a negative result will throw.
  AIndex about_to_be_negative_index(0);
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      about_to_be_negative_index--,
      "Post-decrementing produced an invalid index.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(AIndex()--,
                                "Post-decrementing an invalid index.+");
}

// Tests integer addition and subtraction.
GTEST_TEST(TypeSafeIndex, AdditionAndSubtraction) {
  // NOTE: The result of binary operations will *always* be an int. To
  // perform these operations, the index is implicitly converted to an int in
  // all cases.
  AIndex index1(8);
  AIndex index2(5);
  EXPECT_EQ(index1 + index2, AIndex(13));
  EXPECT_EQ(index1 - index2, AIndex(3));
  // A negative result would be an invalid index, but it *is* a valid int.
  EXPECT_EQ(index2 - index1, -3);
}

// Tests in-place addition.
GTEST_TEST(TypeSafeIndex, InPlaceAddition) {
  AIndex index(8);
  AIndex index_plus_seven = index += 7;
  EXPECT_EQ(index, AIndex(15));
  EXPECT_EQ(index_plus_seven, AIndex(15));
  EXPECT_EQ(index, index_plus_seven);

  // In Debug builds additions leading to a negative result will throw.

  // In-place with an int.
  AIndex about_to_be_negative_index(7);
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      about_to_be_negative_index += (-9),
      "In-place addition with an int produced an invalid index.+");
  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      invalid += 1,
      "In-place addition with an int on an invalid index.+");

  // In-place with an index.
  AIndex max_index(std::numeric_limits<int>::max());
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      max_index += AIndex(1),
      "In-place addition with another index produced an invalid index.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      invalid += AIndex(1),
      "In-place addition with another index invalid LHS.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      index += invalid,
      "In-place addition with another index invalid RHS.+");
}

// Tests in-place subtraction.
GTEST_TEST(TypeSafeIndex, InPlaceSubtract) {
  AIndex index(8);
  AIndex index_minus_seven = index -= 7;
  EXPECT_EQ(index, AIndex(1));
  EXPECT_EQ(index_minus_seven, AIndex(1));
  EXPECT_EQ(index, index_minus_seven);

  // In Debug builds decrements leading to a negative result will throw.

  // In-place with an int.
  AIndex about_to_be_negative_index(0);
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= 7,
      "In-place subtraction with an int produced an invalid index.+");
  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      invalid -= -3,
      "In-place subtraction with an int on an invalid index.+");

  // In-place with an index.
  about_to_be_negative_index = 0;
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= AIndex(1),
      "In-place subtraction with another index produced an invalid index.+");
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      invalid -= AIndex(1),
      "In-place subtraction with another index invalid LHS.+");
  about_to_be_negative_index = 0;
  EXPECT_ERROR_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= invalid,
      "In-place subtraction with another index invalid RHS.+");
}

// Tests stream insertion.
GTEST_TEST(TypeSafeIndex, StreamInsertion) {
  AIndex index(8);
  std::stringstream stream;
  stream << index;
  EXPECT_EQ(stream.str(), "8");

  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(stream << invalid,
                                "Converting to an int.+");
}

// Tests conversion to string via std::to_string function.
GTEST_TEST(TypeSafeIndex, ToString) {
  const int value = 17;
  AIndex index(value);
  EXPECT_EQ(std::to_string(index), std::to_string(value));

  AIndex invalid;
  EXPECT_ERROR_MESSAGE_IF_ARMED(std::to_string(invalid),
                                "Converting to an int.+");
}

// Verifies that it is not possible to convert between two different
// index types.
GTEST_TEST(TypeSafeIndex, ConversionNotAllowedBetweenDifferentTypes) {
  // Conversion is not allowed between two different index types.
  // Note: the extra set of parentheses are needed to avoid the test macro
  // getting confused with the comma inside the template brackets.
  EXPECT_FALSE((std::is_convertible<AIndex, BIndex>::value));
  // The trivial case of course is true.
  EXPECT_TRUE((std::is_convertible<AIndex, AIndex>::value));
  EXPECT_TRUE((std::is_convertible<BIndex, BIndex>::value));
}

// Exercises the index in an STL context. This isn't intended to be exhaustive,
// merely the canary in the coal mine.
GTEST_TEST(TypeSafeIndex, UseInStl) {
  std::vector<AIndex> indices;
  EXPECT_NO_THROW(indices.resize(3));
  EXPECT_FALSE(indices[0].is_valid());
  EXPECT_NO_THROW(indices[0] = 0);
  EXPECT_NO_THROW(indices[1] = AIndex(1));
  EXPECT_NO_THROW(indices[2] = AIndex());  // Valid for *move* assignment.
  AIndex invalid;
  EXPECT_NO_THROW(indices[2] = invalid);
  EXPECT_NO_THROW(indices.emplace_back(3));
  EXPECT_NO_THROW(indices.emplace_back(AIndex(4)));
  EXPECT_NO_THROW(indices.emplace_back());
  EXPECT_NO_THROW(indices.emplace_back(AIndex()));
}

//-------------------------------------------------------------------

// This code allows us to turn compile-time errors in to run-time errors that
// we can incorporate in a unit test.  We can use it to confirm the
// non-existence of a particular operator.  Specifically, we are using it to
// confirm that an index with one tag type cannot be compared or combined with
// index instances of another type. We are also confirming that those same
// operations work with atomic data types that can be converted to int types.
//
// To simplify the test boilerplate, the infrastructure has been placed in a
// macro, allowing for the test of a wide range of *binary* operations.  The
// use is:
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
    typename = decltype(std::declval<T>() OP std::declval<U>())> \
bool has_ ## OP_NAME ## _helper(int) { return true; } \
template <typename T, typename U> \
bool has_ ## OP_NAME ## _helper(...) { return false; } \
template <typename T, typename U> \
bool has_ ## OP_NAME() { return has_ ## OP_NAME ## _helper<T, U>(1); } \
GTEST_TEST(TypeSafeIndex, OP_NAME ## OperatorAvailiblity) { \
  EXPECT_FALSE((has_ ## OP_NAME<AIndex, BIndex>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AIndex, AIndex>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AIndex, int>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AIndex, size_t>())); \
  EXPECT_TRUE((has_ ## OP_NAME<AIndex, int64_t>())); \
}

//-------------------------------------------------------------------

// Confirms that indices of different tag types cannot be compared for equality.
BINARY_TEST(==, Equals)

// Confirms that indices of different tag types cannot be compared for
// inequality.
BINARY_TEST(!=, NotEquals)

// Confirms that indices of different tag types cannot be compared as one less
// than the other.
BINARY_TEST(<, LessThan)

// Confirms that indices of different tag types cannot be compared as one less
// than or equal to the other.
BINARY_TEST(<=, LessThanOrEquals)

// Confirms that indices of different tag types cannot be compared as one
// greater than the other.
BINARY_TEST(>, GreaterThan)

// Confirms that indices of different tag types cannot be compared as one
// greater than or equal to the other.
BINARY_TEST(>=, GreaterThanOrEqual)

// Confirms that indices of different tag types cannot be added to each other.
BINARY_TEST(+=, InPlaceAdd)

// Confirms that indices of different tag types cannot be added to each other.
BINARY_TEST(-=, InPlaceSubtract)

// Confirms that one index cannot be assigned to by another index type (but int
// types and same index types can). This is partially redundant to the
// assignment test above, but the redundancy doesn't hurt.
BINARY_TEST(=, Assignment)

// This tests that one index cannot be *constructed* from another index type,
// but can be constructed from int types.
template <typename T, typename U, typename = decltype(T(U()))>
bool has_construct_helper(int) { return true; }
template <typename T, typename U>
bool has_construct_helper(...) { return false; }
template <typename T, typename U>
bool has_constructor() { return has_construct_helper<T, U>(1); }
GTEST_TEST(TypeSafeIndex, ConstructorAvailability) {
  EXPECT_FALSE((has_constructor<AIndex, BIndex>()));
  EXPECT_TRUE((has_constructor<AIndex, int>()));
  EXPECT_TRUE((has_constructor<AIndex, size_t>()));
  EXPECT_TRUE((has_constructor<AIndex, int64_t>()));
}

// Confirms that type safe indexes are configured to serve as key and/or values
// within std::unordered_map
GTEST_TEST(TypeSafeIndex, CompatibleWithUnorderedMap) {
  std::unordered_map<AIndex, std::string> indexes;
  AIndex a1(1), a2(2), a3(3);
  std::string s1("hello"), s2("unordered"), s3("map");
  indexes.emplace(a1, s1);
  indexes.emplace(a2, s2);
  EXPECT_EQ(indexes.find(a3), indexes.end());
  EXPECT_NE(indexes.find(a2), indexes.end());
  EXPECT_NE(indexes.find(a1), indexes.end());
  indexes[a3] = s3;
  EXPECT_NE(indexes.find(a3), indexes.end());
}

// Confirms that type safe indexes are configured to serve as values
// within std::unordered_set
GTEST_TEST(TypeSafeIndex, CompatibleWithUnorderedSet) {
  std::unordered_set<AIndex> indexes;
  AIndex a1(1), a2(2), a3(3);

  indexes.emplace(a1);
  indexes.insert(a2);
  EXPECT_EQ(indexes.size(), 2);
  EXPECT_EQ(indexes.find(a3), indexes.end());
  EXPECT_NE(indexes.find(a1), indexes.end());
  EXPECT_NE(indexes.find(a2), indexes.end());
}

}  // namespace
}  // namespace common
}  // namespace drake
