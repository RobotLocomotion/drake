#include "drake/common/type_safe_index.h"

#include <regex>
#include <sstream>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

namespace drake {
namespace common {
namespace {

using std::regex;
using std::regex_match;

#ifndef DRAKE_ASSERT_IS_DISARMED
#define EXPECT_THROW_IF_ARMED(expr) EXPECT_THROW(expr, std::runtime_error);
#else
#define EXPECT_THROW_IF_ARMED(expr)
#endif

// Create dummy index types to exercise the functionality
using AIndex = TypeSafeIndex<class A>;
using BIndex = TypeSafeIndex<class B>;

// Verifies the constructor behavior -- in debug and release modes.
GTEST_TEST(TypeSafeIndex, Constructor) {
  AIndex index(1);
  EXPECT_EQ(index, 1);  // This also tests operator==(int).
  AIndex index2(index);  // Copy constructor.
  EXPECT_EQ(index2, 1);
}

// Verifies implicit conversion from index to int.
GTEST_TEST(TypeSafeIndex, ConversionToInt) {
  AIndex index(4);
  int four = index;
  EXPECT_EQ(four, index);
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

// Tests the prefix increment behavior.
GTEST_TEST(TypeSafeIndex, PrefixIncrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_plus = ++index;
  EXPECT_EQ(index, AIndex(9));
  EXPECT_EQ(index_plus, AIndex(9));
}

// Tests the postfix increment behavior.
GTEST_TEST(TypeSafeIndex, PostfixIncrement) {
  AIndex index(8);
  EXPECT_EQ(index, AIndex(8));
  AIndex index_plus = index++;
  EXPECT_EQ(index, AIndex(9));
  EXPECT_EQ(index_plus, AIndex(8));
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
  EXPECT_THROW_IF_ARMED(--about_to_be_negative_index);
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
  EXPECT_THROW_IF_ARMED(about_to_be_negative_index--);
}

// Tests integer addition and subtraction.
GTEST_TEST(TypeSafeIndex, AdditionAndSubtraction) {
  AIndex index1(8);
  AIndex index2(5);
  EXPECT_EQ(index1 + index2, AIndex(13));
  EXPECT_EQ(index1 - index2, AIndex(3));
  // Negative results are an int, not an index, and therefore are allowed.
  EXPECT_EQ(index2 - index1, -3);
  // However construction from a negative result is not allowed.
  EXPECT_THROW_IF_ARMED(AIndex bad_index(index2 - index1));
}

// Tests in-place addition.
GTEST_TEST(TypeSafeIndex, InPlaceAddition) {
  AIndex index(8);
  AIndex index_plus_seven = index += 7;
  EXPECT_EQ(index, AIndex(15));
  EXPECT_EQ(index_plus_seven, AIndex(15));
  EXPECT_EQ(index, index_plus_seven);
  // In Debug builds additions leading to a negative result will throw.
  AIndex about_to_be_negative_index(7);
  EXPECT_THROW_IF_ARMED(about_to_be_negative_index += (-9));
}

// Tests in-place subtraction.
GTEST_TEST(TypeSafeIndex, InPlaceSubtract) {
  AIndex index(8);
  AIndex index_minus_seven = index -= 7;
  EXPECT_EQ(index, AIndex(1));
  EXPECT_EQ(index_minus_seven, AIndex(1));
  EXPECT_EQ(index, index_minus_seven);
  // In Debug builds decrements leading to a negative result will throw.
  AIndex about_to_be_negative_index(0);
  EXPECT_THROW_IF_ARMED(about_to_be_negative_index -= 7);
}

// Tests stream insertion.
GTEST_TEST(TypeSafeIndex, StreamInsertion) {
  AIndex index(8);
  std::stringstream stream;
  stream << index;
  EXPECT_EQ(stream.str(), "8");
}

// Tests conversion to string via std::to_string function.
GTEST_TEST(TypeSafeIndex, ToString) {
  const int value = 17;
  AIndex index(value);
  EXPECT_EQ(std::to_string(index), std::to_string(value));
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

// Verifies the behavior of the assignment operator:
//  - One index can be assigned to another (if they're the same type).
//  - Int types are correctly assigned to the index.
//  - Negative values are caught in debug builds.
GTEST_TEST(TypeSafeIndex, ValueAssignment) {
  AIndex index(0);
  AIndex target(3);
  EXPECT_EQ(target, 3);
  target = index;
  EXPECT_EQ(target, 0);
  target = 17;
  EXPECT_EQ(target, 17);
  EXPECT_THROW_IF_ARMED(target = -1);
}

//-------------------------------------------------------------------

// This codes tests the operations on invalid indices. In release mode, the
// operations are "valid", but in debug they throw an exception. This is *not*
// all inclusive. Some invalid tests are included in the previous operator-
// specific tests, encapsulated within EXPECT_THROW_IF_ARMED.

#ifdef DRAKE_ASSERT_IS_DISARMED
// Performs operations on an invalid index in Release mode. None of these should
// throw an exception.
GTEST_TEST(TypeSafeIndex, OperationOnInvalidInRelease) {
  AIndex invalid(-1);
  AIndex valid(1);
  // Confirm it reports as invalid.
  EXPECT_FALSE(invalid.is_valid());
  // Implicit conversion to int.
  EXPECT_EQ(invalid, -1);
  // Assignment.
  EXPECT_EQ(invalid = -1, -1);
  // Comparison operators.
  EXPECT_FALSE(invalid == valid);
  EXPECT_FALSE(valid == invalid);
  EXPECT_TRUE(invalid != valid);
  EXPECT_TRUE(valid != invalid);
  EXPECT_TRUE(invalid < valid);
  EXPECT_FALSE(valid < invalid);
  EXPECT_TRUE(invalid <= valid);
  EXPECT_FALSE(valid <= invalid);
  EXPECT_FALSE(invalid > valid);
  EXPECT_TRUE(valid > invalid);
  EXPECT_FALSE(invalid >= valid);
  EXPECT_TRUE(valid >= invalid);
  // Pre-decrement/increment operations.
  EXPECT_FALSE((--invalid).is_valid());
  EXPECT_EQ(invalid, -2);
  EXPECT_FALSE((++invalid).is_valid());
  EXPECT_EQ(invalid, -1);
  // Post-decrement/increment operations.
  EXPECT_FALSE((invalid--).is_valid());
  EXPECT_EQ(invalid, -2);
  EXPECT_FALSE((invalid++).is_valid());
  EXPECT_EQ(invalid, -1);
  // In-place arithmetic operations.
  EXPECT_EQ(invalid += invalid, -2);
  EXPECT_EQ(invalid += -1, -3);
  EXPECT_EQ(invalid -= -1, -2);
  EXPECT_EQ(invalid -= AIndex(), -1);
}
#else

// Helper macro for "expecting" an exception but *also* testing the error
// message against the provided regular expression.
#define EXPECT_ERROR_MESSAGE(expression, exception, reg_exp) \
try { \
  expression; \
  GTEST_FAIL(); \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const char* re) { \
    return regex_match(s, regex(re)); }; \
  EXPECT_PRED2(matcher, err.what(), reg_exp); \
}

// Performs operations on an invalid index in Debug mode. All of these should
// throw exceptions with meaningful error messages.
GTEST_TEST(TypeSafeIndex, OperationOnInvalidInDebug) {
  int i;
  AIndex valid(1);
  AIndex invalid;
  EXPECT_FALSE(invalid.is_valid());
  // Explicit construction with negative value.
  EXPECT_ERROR_MESSAGE({ AIndex(-1); }, std::runtime_error,
                       "Constructing an invalid index."
                       ".+negative value = \\-\\d+.+");

  // Implicit conversion to int.
  EXPECT_ERROR_MESSAGE({ i = invalid; }, std::runtime_error,
                       "Converting to an int."
                       ".+negative value = \\-\\d+.+");
  // Assignment.
  EXPECT_ERROR_MESSAGE({ invalid = -1; }, std::runtime_error,
                       "Assigning an invalid int."
                       ".+negative value = \\-\\d+.+");

  // This is a no-op function so the compiler doesn't think I'm computing
  // unused boolean values in the next set of comparison tests.
  auto consume = [](bool) {};

  // Comparison operators.
  EXPECT_ERROR_MESSAGE(consume(invalid == valid), std::runtime_error,
                       "Testing == with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid == invalid), std::runtime_error,
                       "Testing == with invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(invalid != valid), std::runtime_error,
                       "Testing != with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid != invalid), std::runtime_error,
                       "Testing != with invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(invalid < valid), std::runtime_error,
                       "Testing < with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid < invalid), std::runtime_error,
                       "Testing < with invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(invalid <= valid), std::runtime_error,
                       "Testing <= with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid <= invalid), std::runtime_error,
                       "Testing <= with invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(invalid > valid), std::runtime_error,
                       "Testing > with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid > invalid), std::runtime_error,
                       "Testing > with invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(invalid >= valid), std::runtime_error,
                       "Testing >= with invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(consume(valid >= invalid), std::runtime_error,
                       "Testing >= with invalid RHS."
                       ".+negative value = \\-\\d+.+");

  // Pre-decrement/increment operations.
  EXPECT_ERROR_MESSAGE(--invalid, std::runtime_error,
                       "Pre-decrementing an invalid index."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(++invalid, std::runtime_error,
                       "Pre-incrementing an invalid index."
                       ".+negative value = \\-\\d+.+");

  // Post-decrement/increment operations.
  EXPECT_ERROR_MESSAGE(invalid--, std::runtime_error,
                       "Post-decrementing an invalid index."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(invalid++, std::runtime_error,
                       "Post-incrementing an invalid index."
                       ".+negative value = \\-\\d+.+");

  // In-place arithmetic operations.
  EXPECT_ERROR_MESSAGE(invalid += 1, std::runtime_error,
                       "In-place addition with an int on an invalid index."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(invalid -= 1, std::runtime_error,
                       "In-place subtraction with an int on an invalid index."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(invalid += valid, std::runtime_error,
                       "In-place addition with another index invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(invalid -= valid, std::runtime_error,
                       "In-place subtraction with another index invalid LHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(valid += invalid, std::runtime_error,
                       "In-place addition with another index invalid RHS."
                       ".+negative value = \\-\\d+.+");
  EXPECT_ERROR_MESSAGE(valid -= invalid, std::runtime_error,
                       "In-place subtraction with another index invalid RHS."
                       ".+negative value = \\-\\d+.+");
}

#endif

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

}  // namespace
}  // namespace common
}  // namespace drake
