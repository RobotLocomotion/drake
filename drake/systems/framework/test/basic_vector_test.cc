#include "drake/systems/framework/basic_vector.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace systems {
namespace {

// Tests that the BasicVector can be mutated in-place.
GTEST_TEST(BasicVectorTest, Mutate) {
  BasicVector<int> vec(2);
  vec.get_mutable_value() << 1, 2;
  Eigen::Vector2i expected;
  expected << 1, 2;
  EXPECT_EQ(expected, vec.get_value());
}

// Tests that the BasicVector can be set from another vector.
GTEST_TEST(BasicVectorTest, SetWholeVector) {
  BasicVector<int> vec(2);
  vec.get_mutable_value() << 1, 2;
  Eigen::Vector2i next_value;
  next_value << 3, 4;
  vec.set_value(next_value);
  EXPECT_EQ(next_value, vec.get_value());
}

// Tests that an error is thrown when the BasicVector is set from a vector
// of a different size.
GTEST_TEST(BasicVectorTest, ReinitializeInvalid) {
  BasicVector<int> vec(2);
  Eigen::Vector3i next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vec.set_value(next_value), std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
