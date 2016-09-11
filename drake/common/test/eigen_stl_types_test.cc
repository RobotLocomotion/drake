#include "drake/common/eigen_stl_types.h"

#include <memory>

#include "gtest/gtest.h"

using Eigen::Vector2d;

namespace drake {
namespace {

// Confirm that nothing asserts nor segfaults.
GTEST_TEST(EigenStlTypesTest, UnorderedMapTest) {
  // The device under test ("dut").
  eigen_aligned_std_unordered_map<int, Vector2d> dut0;
  dut0[0] = Vector2d::Zero();
  auto dut1 =
      std::make_unique<eigen_aligned_std_unordered_map<int, Vector2d>>();
  dut1->insert(std::make_pair(1, Vector2d::Ones()));

  EXPECT_TRUE(true);
}

}  // namespace
}  // namespace drake
