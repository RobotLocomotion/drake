#include "drake/systems/framework/named_value_vector.h"

#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(NamedValueVectorTest, Access) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(2, *vector.get_named_value("bar"));
  EXPECT_EQ(2, vector.size());
  Eigen::Matrix<int, 2, 1> expected;
  expected << 1, 2;
  EXPECT_EQ(expected, vector.get_value());
}

GTEST_TEST(NamedValueVectorTest, Set) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  vector.set_named_value("bar", 5);
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(5, *vector.get_named_value("bar"));
}

// Tests that setting a name that doesn't exist throws an exception.
GTEST_TEST(NamedValueVectorTest, SetInvalidKey) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  EXPECT_THROW(vector.set_named_value("baz", 5), std::runtime_error);
}

// Tests that getting a name that doesn't exist returns nullptr.
GTEST_TEST(NamedValueVectorTest, GetInvalidKey) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  EXPECT_EQ(nullptr, vector.get_named_value("baz"));
}

GTEST_TEST(NamedValueVectorTest, Mutate) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  vector.get_mutable_value()[1] = 6;
  EXPECT_EQ(1, *vector.get_named_value("foo"));
  EXPECT_EQ(6, *vector.get_named_value("bar"));
}

GTEST_TEST(NamedValueVectorTest, SetWholeVector) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  Eigen::Matrix<int, 2, 1> next_value;
  next_value << 3, 4;
  vector.set_value(next_value);
  EXPECT_EQ(3, *vector.get_named_value("foo"));
  EXPECT_EQ(4, *vector.get_named_value("bar"));
}

// Tests that an error is thrown when the NamedValueVector is set to a vector
// of a different size.
GTEST_TEST(NamedValueVectorTest, ReinitializeInvalid) {
  NamedValueVector<int> vector(
      std::vector<std::pair<std::string, int>>{{"foo", 1}, {"bar", 2}});
  Eigen::Matrix<int, 3, 1> next_value;
  next_value << 3, 4, 5;
  EXPECT_THROW(vector.set_value(next_value), std::out_of_range);
}

// Tests that an error is thrown when there are multiple identically
// named elements.
GTEST_TEST(NamedValueVectorTest, InvalidNames) {
  std::unique_ptr<NamedValueVector<int>> vector;
  EXPECT_THROW(vector.reset(new NamedValueVector<int>(
                   std::vector<std::string>{"foo", "foo"})),
               std::runtime_error);
}

}  // namespace
}  // namespace systems
}  // namespace drake
