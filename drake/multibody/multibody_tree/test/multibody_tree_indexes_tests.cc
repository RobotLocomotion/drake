#include "gtest/gtest.h"

#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include <string>
#include <sstream>

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
    IndexType index_plus_seven = index -= 7;
    EXPECT_EQ(index, IndexType(1));
    EXPECT_EQ(index_plus_seven, IndexType(1));
    EXPECT_EQ(index, index_plus_seven);
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

}  // namespace
}  // namespace multibody
}  // namespace drake
