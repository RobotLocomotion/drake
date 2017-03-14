#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

#include "gtest/gtest.h"

namespace drake {
namespace multibody {
namespace {

// Verifies the correct behavior of BodyIndex.
GTEST_TEST(MultibodyTreeIndexes, BodyIndex) {
  // Verify the we can retrieve the "world" id.
  EXPECT_EQ(world_index(), BodyIndex(0));
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
