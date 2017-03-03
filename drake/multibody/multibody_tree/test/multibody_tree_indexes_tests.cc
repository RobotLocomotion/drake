#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include <sstream>
#include <type_traits>

#include "gtest/gtest.h"

namespace drake {
namespace multibody {
namespace {

template <class IndexType>
void RunMultibodyIndexTests() {
  // Construction from an int.
  {
    IndexType index(1);
    EXPECT_EQ(index, 1);  // This also tests operator==(int).
  }

  // Default construction, makes a zero index.
  {
    IndexType index;
    EXPECT_EQ(index, 0);
  }

  // Conversion operator.
  {
    IndexType index(4);
    int four = index;
    EXPECT_EQ(four, index);
  }

  // Comparison operators.
  {
    IndexType index1(5);
    IndexType index2(5);
    IndexType index3(7);
    EXPECT_EQ(index1, index2);  // operator==
    EXPECT_NE(index1, index3);  // operator!=
    EXPECT_LT(index1, index3);  // operator<
    EXPECT_LE(index1, index2);  // operator<=
    EXPECT_GT(index3, index1);  // operator>
    EXPECT_GE(index2, index1);  // operator>=
  }

  // Prefix increment.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_plus = ++index;
    EXPECT_EQ(index, IndexType(9));
    EXPECT_EQ(index_plus, IndexType(9));
  }

  // Postfix increment.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_plus = index++;
    EXPECT_EQ(index, IndexType(9));
    EXPECT_EQ(index_plus, IndexType(8));
  }

  // Prefix decrement.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_minus = --index;
    EXPECT_EQ(index, IndexType(7));
    EXPECT_EQ(index_minus, IndexType(7));
  }

  // Postfix decrement.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_minus = index--;
    EXPECT_EQ(index, IndexType(7));
    EXPECT_EQ(index_minus, IndexType(8));
  }

  // Addition and Subtraction.
  {
    IndexType index1(8);
    IndexType index2(5);
    EXPECT_EQ(index1 + index2, IndexType(13));
    EXPECT_EQ(index1 - index2, IndexType(3));
  }

  // Addition assignment.
  {
    IndexType index(8);
    IndexType index_plus_seven = index += 7;
    EXPECT_EQ(index, IndexType(15));
    EXPECT_EQ(index_plus_seven, IndexType(15));
    EXPECT_EQ(index, index_plus_seven);
  }

  // Subtraction assignment.
  {
    IndexType index(8);
    IndexType index_minus_seven = index -= 7;
    EXPECT_EQ(index, IndexType(1));
    EXPECT_EQ(index_minus_seven, IndexType(1));
    EXPECT_EQ(index, index_minus_seven);
  }

  // Stream insertion operator.
  {
    IndexType index(8);
    std::stringstream stream;
    stream << index;
    EXPECT_EQ(stream.str(), "8");
  }
}

// Verifies the correct behavior of FrameIndex.
GTEST_TEST(MultibodyTreeIndexes, FrameIndex) {
  RunMultibodyIndexTests<FrameIndex>();
}

// Verifies the correct behavior of BodyIndex.
GTEST_TEST(MultibodyTreeIndexes, BodyIndex) {
  RunMultibodyIndexTests<BodyIndex>();
}

// Verifies the correct behavior of MobilizerIndex.
GTEST_TEST(MultibodyTreeIndexes, MobilizerIndex) {
  RunMultibodyIndexTests<MobilizerIndex>();
}

// Verifies that it is not possible to convert between two different
// index types.
GTEST_TEST(MultibodyTreeIndexes, ConversionNotAllowedBetweenDifferentTypes) {
  // Conversion is not allowed between two different index types.
  // Note: the extra set of parentheses are needed to avoid the test macro
  // getting confused with the comma inside the template brackets.
  EXPECT_FALSE((std::is_convertible<BodyIndex, FrameIndex>::value));
  // The trivial case of course is true.
  EXPECT_TRUE((std::is_convertible<BodyIndex, BodyIndex>::value));
  EXPECT_TRUE((std::is_convertible<FrameIndex, FrameIndex>::value));
}


}  // namespace
}  // namespace multibody
}  // namespace drake
