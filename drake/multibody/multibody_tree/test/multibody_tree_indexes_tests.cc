#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include <sstream>
#include <type_traits>

#include "gtest/gtest.h"

#include "drake/common/nice_type_name.h"

namespace drake {
namespace multibody {
namespace {

#ifndef DRAKE_ASSERT_IS_DISARMED
#define EXPECT_THROW_IF_ARMED(expr) EXPECT_THROW(expr, std::runtime_error);
#else
#define EXPECT_THROW_IF_ARMED(expr)
#endif

template <class IndexType>
void RunMultibodyIndexTests() {
  // Construction from an int.
  {
    IndexType index(1);
    EXPECT_EQ(index, 1);  // This also tests operator==(int).
    // In Debug builds construction from a negative int aborts.
#ifndef DRAKE_ASSERT_IS_DISARMED
    try {
      IndexType negative_index(-1);
      GTEST_FAIL();
    } catch (std::runtime_error& e) {
      std::string expected_msg =
          "This index, of type \"" + drake::NiceTypeName::Get<IndexType>() +
          "\", has the negative value = -1. Negative indexes are not allowed";
      EXPECT_EQ(e.what(), expected_msg);
    }
#endif
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
    // In Debug builds decrements leading to a negative result will throw.
    IndexType about_to_be_negative_index(0);
    EXPECT_THROW_IF_ARMED(--about_to_be_negative_index);
  }

  // Postfix decrement.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_minus = index--;
    EXPECT_EQ(index, IndexType(7));
    EXPECT_EQ(index_minus, IndexType(8));
    // In Debug builds decrements leading to a negative result will throw.
    IndexType about_to_be_negative_index(0);
    EXPECT_THROW_IF_ARMED(about_to_be_negative_index--);
  }

  // Addition and Subtraction.
  {
    IndexType index1(8);
    IndexType index2(5);
    EXPECT_EQ(index1 + index2, IndexType(13));
    EXPECT_EQ(index1 - index2, IndexType(3));
    // Negative results are an int, not an index, and therefore are allowed.
    EXPECT_EQ(index2 - index1, -3);
    // However construction from a negative result is not allowed.
    EXPECT_THROW_IF_ARMED(IndexType bad_index(index2 - index1));
  }

  // Addition assignment.
  {
    IndexType index(8);
    IndexType index_plus_seven = index += 7;
    EXPECT_EQ(index, IndexType(15));
    EXPECT_EQ(index_plus_seven, IndexType(15));
    EXPECT_EQ(index, index_plus_seven);
    // In Debug builds additions leading to a negative result will throw.
    IndexType about_to_be_negative_index(7);
    EXPECT_THROW_IF_ARMED(about_to_be_negative_index += (-9));
  }

  // Subtraction assignment.
  {
    IndexType index(8);
    IndexType index_minus_seven = index -= 7;
    EXPECT_EQ(index, IndexType(1));
    EXPECT_EQ(index_minus_seven, IndexType(1));
    EXPECT_EQ(index, index_minus_seven);
    // In Debug builds decrements leading to a negative result will throw.
    IndexType about_to_be_negative_index(0);
    EXPECT_THROW_IF_ARMED(about_to_be_negative_index -= 7);
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
  // Verify the we can retrieve the "world" id.
  EXPECT_EQ(world_index(), BodyIndex(0));
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
