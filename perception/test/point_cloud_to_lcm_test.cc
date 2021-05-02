#include "drake/perception/point_cloud_to_lcm.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/lcmt_point_cloud.hpp"
#include "drake/perception/point_cloud.h"

using drake::Vector3;
using Eigen::Vector3f;

namespace drake {
namespace perception {
namespace {

// Convert a small, XYZ-only cloud with a few non-finite points.
GTEST_TEST(PointCloudToLcmTest, XyzOnly) {
  constexpr float kInf = std::numeric_limits<float>::infinity();

  // Create the input.
  PointCloud cloud(5);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(NAN, 0.0, 0.0);
  cloud.mutable_xyz(2) = Vector3f(0.0, kInf, 0.0);
  cloud.mutable_xyz(3) = Vector3f(0.0, 0.0, kInf);
  cloud.mutable_xyz(4) = Vector3f(4.0, 5.0, 6.0);
  const int finite_count = 2;

  // Feed through the device under test.
  PointCloudToLcm dut;
  auto context = dut.CreateDefaultContext();
  context->SetTime(1.0);
  dut.get_input_port().FixValue(context.get(), Value<PointCloud>(cloud));
  const auto& output = dut.get_output_port().Eval<lcmt_point_cloud>(*context);

  // Confirm all message fields.
  EXPECT_EQ(output.header.seq, 0);
  EXPECT_EQ(output.header.utime, 1'000'000);
  EXPECT_EQ(output.header.frame_name, "");
  ASSERT_EQ(output.num_points, finite_count);
  ASSERT_EQ(output.points.size(), 3);
  ASSERT_EQ(output.points[0].size(), finite_count);
  ASSERT_EQ(output.points[1].size(), finite_count);
  ASSERT_EQ(output.points[2].size(), finite_count);
  EXPECT_EQ(output.points[0][0], 1.0);
  EXPECT_EQ(output.points[1][0], 2.0);
  EXPECT_EQ(output.points[2][0], 3.0);
  EXPECT_EQ(output.points[0][1], 4.0);
  EXPECT_EQ(output.points[1][1], 5.0);
  EXPECT_EQ(output.points[2][1], 6.0);
  EXPECT_EQ(output.num_channels, 0);
  EXPECT_EQ(output.channel_names.size(), 0);
  EXPECT_EQ(output.channels.size(), 0);
}

// Convert a small, XYZ+RGB cloud with one non-finite point.
GTEST_TEST(PointCloudToLcmTest, XyzRgb) {
  // Create the input.
  PointCloud cloud(3, pc_flags::kXYZs | pc_flags::kRGBs);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(NAN, 0.0, 0.0);
  cloud.mutable_xyz(2) = Vector3f(4.0, 5.0, 6.0);
  cloud.mutable_rgb(0) = Vector3<uint8_t>(0, 127, 255);
  cloud.mutable_rgb(1) = Vector3<uint8_t>(0, 0, 0);
  cloud.mutable_rgb(2) = Vector3<uint8_t>(1, 128, 255);
  const int finite_count = 2;

  // Feed through the device under test.
  PointCloudToLcm dut;
  auto context = dut.CreateDefaultContext();
  context->SetTime(1.0);
  dut.get_input_port().FixValue(context.get(), Value<PointCloud>(cloud));
  const auto& output = dut.get_output_port().Eval<lcmt_point_cloud>(*context);

  // Confirm all message fields.
  EXPECT_EQ(output.header.seq, 0);
  EXPECT_EQ(output.header.utime, 1'000'000);
  EXPECT_EQ(output.header.frame_name, "");
  ASSERT_EQ(output.num_points, finite_count);
  ASSERT_EQ(output.points.size(), 3);
  ASSERT_EQ(output.points[0].size(), finite_count);
  ASSERT_EQ(output.points[1].size(), finite_count);
  ASSERT_EQ(output.points[2].size(), finite_count);
  EXPECT_EQ(output.points[0][0], 1.0);
  EXPECT_EQ(output.points[1][0], 2.0);
  EXPECT_EQ(output.points[2][0], 3.0);
  EXPECT_EQ(output.points[0][1], 4.0);
  EXPECT_EQ(output.points[1][1], 5.0);
  EXPECT_EQ(output.points[2][1], 6.0);
  ASSERT_EQ(output.num_channels, 3);
  ASSERT_EQ(output.channel_names.size(), 3);
  ASSERT_EQ(output.channels.size(), 3);
  ASSERT_EQ(output.channels[0].size(), finite_count);
  ASSERT_EQ(output.channels[1].size(), finite_count);
  ASSERT_EQ(output.channels[2].size(), finite_count);
  EXPECT_EQ(output.channel_names[0], "r");
  EXPECT_EQ(output.channel_names[1], "g");
  EXPECT_EQ(output.channel_names[2], "b");
  EXPECT_EQ(output.channels[0][0] * 255.0f, 0);
  EXPECT_EQ(output.channels[1][0] * 255.0f, 127);
  EXPECT_EQ(output.channels[2][0] * 255.0f, 255);
  EXPECT_EQ(output.channels[0][1] * 255.0f, 1);
  EXPECT_EQ(output.channels[1][1] * 255.0f, 128);
  EXPECT_EQ(output.channels[2][1] * 255.0f, 255);
}

}  // namespace
}  // namespace perception
}  // namespace drake
