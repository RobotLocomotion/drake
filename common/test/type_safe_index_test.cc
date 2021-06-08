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

#include "drake/common/sorted_pair.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

namespace drake {
namespace common {
namespace {

using std::regex;
using std::regex_match;
using std::move;

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
  AIndex invalid;  // Default constructor.
  EXPECT_FALSE(invalid.is_valid());
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      AIndex(-1), std::runtime_error,
      "Explicitly constructing an invalid index.+");
  DRAKE_EXPECT_NO_THROW(
      unused(AIndex(invalid)));  // Copy construct invalid index.
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

  // Assignment involving invalid indices.

  // Copy assign *to* invalid.
  {
    AIndex source(3);
    AIndex starts_invalid;
    DRAKE_EXPECT_NO_THROW(starts_invalid = source);
    EXPECT_EQ(source, 3);
    EXPECT_EQ(starts_invalid, source);
  }

  // Copy assign *from* invalid.
  {
    AIndex invalid;
    AIndex target(3);
    DRAKE_EXPECT_NO_THROW(target = invalid);
    EXPECT_FALSE(target.is_valid());
  }

  // Move assign *to* invalid.
  {
    AIndex invalid;
    AIndex target(2);
    EXPECT_TRUE(target.is_valid());
    DRAKE_EXPECT_NO_THROW(invalid = move(target));
    EXPECT_FALSE(target.is_valid());
    EXPECT_EQ(invalid, 2);
  }

  // Move assign *from* invalid.
  {
    AIndex invalid;
    AIndex target(3);
    DRAKE_EXPECT_NO_THROW(target = move(invalid));
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(static_cast<int>(invalid)),
                                      std::runtime_error,
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid == valid),
                                      std::runtime_error,
                                      "Testing == with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid == invalid),
                                      std::runtime_error,
                                      "Testing == with invalid RHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid != valid),
                                      std::runtime_error,
                                      "Testing != with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid != invalid),
                                      std::runtime_error,
                                      "Testing != with invalid RHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid < valid),
                                      std::runtime_error,
                                      "Testing < with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid < invalid),
                                      std::runtime_error,
                                      "Testing < with invalid RHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid <= valid),
                                      std::runtime_error,
                                      "Testing <= with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid <= invalid),
                                      std::runtime_error,
                                      "Testing <= with invalid RHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid > valid),
                                      std::runtime_error,
                                      "Testing > with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid > invalid),
                                      std::runtime_error,
                                      "Testing > with invalid RHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(invalid >= valid),
                                      std::runtime_error,
                                      "Testing >= with invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(unused(valid >= invalid),
                                      std::runtime_error,
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      ++max_index, std::runtime_error,
      "Pre-incrementing produced an invalid index.+");
  // Increment invalid index.
  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(++invalid, std::runtime_error,
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      max_index++, std::runtime_error,
      "Post-incrementing produced an invalid index.+");
  // Increment invalid index.
  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(invalid++, std::runtime_error,
                                      "Post-incrementing an invalid index.+");
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      --about_to_be_negative_index, std::runtime_error,
      "Pre-decrementing produced an invalid index.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(--AIndex(), std::runtime_error,
                                      "Pre-decrementing an invalid index.+");
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      about_to_be_negative_index--, std::runtime_error,
      "Post-decrementing produced an invalid index.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(AIndex()--, std::runtime_error,
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      about_to_be_negative_index += (-9), std::runtime_error,
      "In-place addition with an int produced an invalid index.+");
  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      invalid += 1, std::runtime_error,
      "In-place addition with an int on an invalid index.+");

  // In-place with an index.
  AIndex max_index(std::numeric_limits<int>::max());
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      max_index += AIndex(1), std::runtime_error,
      "In-place addition with another index produced an invalid index.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      invalid += AIndex(1), std::runtime_error,
      "In-place addition with another index invalid LHS.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      index += invalid, std::runtime_error,
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
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= 7, std::runtime_error,
      "In-place subtraction with an int produced an invalid index.+");
  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      invalid -= -3, std::runtime_error,
      "In-place subtraction with an int on an invalid index.+");

  // In-place with an index.
  about_to_be_negative_index = AIndex(0);
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= AIndex(1), std::runtime_error,
      "In-place subtraction with another index produced an invalid index.+");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      invalid -= AIndex(1), std::runtime_error,
      "In-place subtraction with another index invalid LHS.+");
  about_to_be_negative_index = AIndex(0);
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      about_to_be_negative_index -= invalid, std::runtime_error,
      "In-place subtraction with another index invalid RHS.+");
}

// Tests stream insertion.
GTEST_TEST(TypeSafeIndex, StreamInsertion) {
  AIndex index(8);
  std::stringstream stream;
  stream << index;
  EXPECT_EQ(stream.str(), "8");

  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(stream << invalid, std::runtime_error,
                                      "Converting to an int.+");
}

// Tests conversion to string via std::to_string function.
GTEST_TEST(TypeSafeIndex, ToString) {
  const int value = 17;
  AIndex index(value);
  EXPECT_EQ(std::to_string(index), std::to_string(value));

  AIndex invalid;
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      std::to_string(invalid), std::runtime_error, "Converting to an int.+");
}

// Verifies that it is not possible to convert between two different
// index types.
GTEST_TEST(TypeSafeIndex, ConversionNotAllowedBetweenDifferentTypes) {
  // Conversion is not allowed between two different index types.
  // Note: the extra set of parentheses are needed to avoid the test macro
  // getting confused with the comma inside the template brackets.
  EXPECT_FALSE((std::is_convertible_v<AIndex, BIndex>));
  // The trivial case of course is true.
  EXPECT_TRUE((std::is_convertible_v<AIndex, AIndex>));
  EXPECT_TRUE((std::is_convertible_v<BIndex, BIndex>));
}

// Exercises the index in an STL context. This isn't intended to be exhaustive,
// merely the canary in the coal mine.
GTEST_TEST(TypeSafeIndex, UseInStl) {
  std::vector<AIndex> indices;
  DRAKE_EXPECT_NO_THROW(indices.resize(3));
  EXPECT_FALSE(indices[0].is_valid());
  DRAKE_EXPECT_NO_THROW(indices[1] = AIndex(1));
  DRAKE_EXPECT_NO_THROW(indices[2] = AIndex());  // Valid for *move* assignment.
  AIndex invalid;
  DRAKE_EXPECT_NO_THROW(indices[2] = invalid);
  DRAKE_EXPECT_NO_THROW(indices.emplace_back(3));
  DRAKE_EXPECT_NO_THROW(indices.emplace_back(AIndex(4)));
  DRAKE_EXPECT_NO_THROW(indices.emplace_back());
  DRAKE_EXPECT_NO_THROW(indices.emplace_back(AIndex()));
}

//-------------------------------------------------------------------

// Helper functions for testing TypeSafeIndex interoperability with certain
// integer types.
template <typename Scalar>
void TestScalarComparisons() {
  Scalar small_value{10};
  Scalar equal_value{15};
  Scalar big_value{20};

  AIndex index{static_cast<int>(equal_value)};

  EXPECT_TRUE(small_value != index);
  EXPECT_TRUE(index != small_value);
  EXPECT_TRUE(equal_value == index);
  EXPECT_TRUE(index == equal_value);

  EXPECT_TRUE(small_value < index);
  EXPECT_TRUE(index < big_value);
  EXPECT_TRUE(small_value <= index);
  EXPECT_TRUE(index <= big_value);
  EXPECT_TRUE(index <= equal_value);

  EXPECT_TRUE(big_value > index);
  EXPECT_TRUE(index > small_value);
  EXPECT_TRUE(big_value >= index);
  EXPECT_TRUE(index >= small_value);
  EXPECT_TRUE(index >= equal_value);
}

template <typename Scalar>
void TestScalarIncrement() {
  const int initial_value = 5;
  AIndex index{initial_value};
  Scalar offset{1};
  index += offset;
  ASSERT_EQ(index, initial_value + offset);
  index -= offset;
  ASSERT_EQ(index, initial_value);
}

// Explicit tests to confirm operations between integral types and indices work
// as expected.

GTEST_TEST(IntegralComparisons, CompareInt) {
  TestScalarComparisons<int>();
  TestScalarIncrement<int>();
  const int value = 3;
  AIndex constructed(value);
  ASSERT_TRUE(constructed.is_valid());
}

GTEST_TEST(IntegralComparisons, CompareInt64) {
  TestScalarComparisons<int64_t>();
  TestScalarIncrement<int64_t>();

  // Implicit narrowing for a valid value.
  const int64_t valid_value = 3;
  AIndex valid(valid_value);
  ASSERT_TRUE(valid.is_valid());

  // Implicit narrowing of value that is too large. In contrast to size_t
  // (see below), the compiler warns about compile-time issues with this
  // assignment. The lambda function provides sufficient indirection that
  // compiler won't warn about the implicit conversion of the too-large int64_t
  // value.
  auto indirect_call = [](const int64_t local_value) {
    AIndex a;
    DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
        a = AIndex(local_value), std::runtime_error,
        "Explicitly constructing an invalid index. Type .* has an invalid "
        "value; it must lie in the range .*");
  };

  // The too-large number, when truncated, would produce a negative index which
  // is inherently invalid.
  const int64_t small_overflow_value = 2300000000;
  EXPECT_LT(static_cast<int>(small_overflow_value), 0);
  indirect_call(small_overflow_value);

  // The too-large number, when truncated, would produce a positive index which
  // is inherently valid; but the pre-truncation value is validated.
  const int64_t big_overflow_value = (1L << 48) | 0x1;
  EXPECT_GT(static_cast<int>(big_overflow_value), 0);
  indirect_call(big_overflow_value);
}

GTEST_TEST(IntegralComparisons, CompareSizeT) {
  TestScalarComparisons<size_t>();
  TestScalarIncrement<size_t>();

  // Implicit narrowing for a valid value.
  const size_t valid_value = 3;
  AIndex valid(valid_value);
  ASSERT_TRUE(valid.is_valid());

  // Implicit narrowing of value that is too large.

  // The too-large number, when truncated, would produce a negative index which
  // is inherently invalid.
  const size_t small_overflow_value = 2300000000;
  EXPECT_LT(static_cast<int>(small_overflow_value), 0);
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      AIndex junk(small_overflow_value), std::runtime_error,
      "Explicitly constructing an invalid index. Type .* has an invalid "
          "value; it must lie in the range .*");

  // The too-large number, when truncated, would produce a positive index which
  // is inherently valid; but the pre-truncation value is validated.
  const size_t big_overflow_value = (1L << 48) | 0x1;
  EXPECT_GT(static_cast<int>(big_overflow_value), 0);
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      AIndex junk(big_overflow_value), std::runtime_error,
      "Explicitly constructing an invalid index. Type .* has an invalid "
          "value; it must lie in the range .*");

  // Now we test right out at the limit of the *smaller* type -- the int.
  const AIndex biggest_index{std::numeric_limits<int>::max()};
  const size_t equal(static_cast<int>(biggest_index));
  const size_t bigger = equal + 1;
  const size_t smaller = equal - 1;

  EXPECT_FALSE(biggest_index == smaller);
  EXPECT_TRUE(biggest_index != smaller);
  EXPECT_FALSE(biggest_index < smaller);
  EXPECT_FALSE(biggest_index <= smaller);
  EXPECT_TRUE(biggest_index > smaller);
  EXPECT_TRUE(biggest_index >= smaller);

  EXPECT_TRUE(biggest_index == equal);
  EXPECT_FALSE(biggest_index != equal);
  EXPECT_FALSE(biggest_index < equal);
  EXPECT_TRUE(biggest_index <= equal);
  EXPECT_FALSE(biggest_index > equal);
  EXPECT_TRUE(biggest_index >= equal);

  EXPECT_FALSE(biggest_index == bigger);
  EXPECT_TRUE(biggest_index != bigger);
  EXPECT_TRUE(biggest_index < bigger);
  EXPECT_TRUE(biggest_index <= bigger);
  EXPECT_FALSE(biggest_index > bigger);
  EXPECT_FALSE(biggest_index >= bigger);
}

// Confirms that comparisons with unsigned types that have fewer bits than
// TypeSafeIndex's underlying int report propertly.
GTEST_TEST(TypeSafeIndex, CompareUnsignedShort) {
  TestScalarComparisons<uint16_t>();
  TestScalarIncrement<uint16_t>();

  // Test right out at the limit of the *smaller* type -- the uint16_t.
  const uint16_t big_unsigned = std::numeric_limits<uint16_t>::max();
  const AIndex bigger_index{static_cast<int>(big_unsigned) + 1};
  const AIndex smaller_index{static_cast<int>(big_unsigned) - 1};
  const AIndex equal_index(static_cast<int>(big_unsigned));

  EXPECT_FALSE(bigger_index == big_unsigned);
  EXPECT_TRUE(bigger_index != big_unsigned);
  EXPECT_FALSE(bigger_index < big_unsigned);
  EXPECT_FALSE(bigger_index <= big_unsigned);
  EXPECT_TRUE(bigger_index > big_unsigned);
  EXPECT_TRUE(bigger_index >= big_unsigned);

  EXPECT_TRUE(equal_index == big_unsigned);
  EXPECT_FALSE(equal_index != big_unsigned);
  EXPECT_FALSE(equal_index < big_unsigned);
  EXPECT_TRUE(equal_index <= big_unsigned);
  EXPECT_FALSE(equal_index > big_unsigned);
  EXPECT_TRUE(equal_index >= big_unsigned);

  EXPECT_FALSE(smaller_index == big_unsigned);
  EXPECT_TRUE(smaller_index != big_unsigned);
  EXPECT_TRUE(smaller_index < big_unsigned);
  EXPECT_TRUE(smaller_index <= big_unsigned);
  EXPECT_FALSE(smaller_index > big_unsigned);
  EXPECT_FALSE(smaller_index >= big_unsigned);
}

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

// Confirms that a SortedPair<IndexType> can be used as a key in a hashing
// container. This is representative of TypeSafeIndex's compatibility with the
// DrakeHash notion.
GTEST_TEST(TypeSafeIndex, SortedPairIndexHashable) {
  AIndex a1(1);
  AIndex a2(2);
  std::unordered_set<SortedPair<AIndex>> pairs;
  pairs.insert({a2, a1});
  EXPECT_EQ(pairs.count(SortedPair<AIndex>(a1, a2)), 1);
}

}  // namespace
}  // namespace common
}  // namespace drake
