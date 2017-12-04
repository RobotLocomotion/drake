#include "drake/common/eigen_stl_types.h"

#include <memory>

#include <gtest/gtest.h>

using Eigen::Vector2d;
using Eigen::Vector4d;

namespace drake {
namespace {

template <typename T>
using UnorderedMapType = eigen_aligned_std_unordered_map<int, T>;

// Confirm that nothing asserts nor segfaults.
GTEST_TEST(EigenStlTypesTest, UnorderedMapTest) {
  // The device under test is abbreviated "dut".
  UnorderedMapType<Vector2d> dut0;
  dut0[0] = Vector2d::Zero();

  auto dut1 = std::make_unique<UnorderedMapType<Vector2d>>();
  dut1->insert(std::make_pair(1, Vector2d::Ones()));

  UnorderedMapType<Vector4d> dut2;
  dut2[2] = Vector4d::Ones();

  auto dut3 = std::make_unique<UnorderedMapType<Vector4d>>();
  dut3->insert(std::make_pair(3, Vector4d::Zero()));

  EXPECT_TRUE(true);
}

template <typename T>
using MapType = eigen_aligned_std_map<int, T>;

// Confirm that nothing asserts nor segfaults.
GTEST_TEST(EigenStlTypesTest, MapTest) {
  // The device under test is abbreviated "dut".
  MapType<Vector2d> dut0;
  dut0[0] = Vector2d::Zero();

  auto dut1 = std::make_unique<MapType<Vector2d>>();
  dut1->insert(std::make_pair(1, Vector2d::Ones()));

  MapType<Vector4d> dut2;
  dut2[2] = Vector4d::Ones();

  auto dut3 = std::make_unique<MapType<Vector4d>>();
  dut3->insert(std::make_pair(3, Vector4d::Zero()));

  EXPECT_TRUE(true);
}

}  // namespace
}  // namespace drake
